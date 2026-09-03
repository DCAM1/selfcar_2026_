#include "vtd_ros2_bridge/rdb_udp_receiver.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>
#include <limits>
#include <utility>

namespace vtd_ros2_bridge {

namespace {

constexpr std::size_t kMaximumUdpDatagramSize = 65536U;
constexpr long kReceiveTimeoutMicroseconds = 250000L;

std::int64_t steady_time_nanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

} // namespace

RdbUdpReceiver::RdbUdpReceiver(std::string bind_address, const int port,
                               DatagramCallback datagram_callback,
                               BindCallback bind_callback,
                               const int socket_receive_buffer_bytes,
                               const std::chrono::milliseconds retry_delay)
    : bind_address_(std::move(bind_address)), port_(port),
      datagram_callback_(std::move(datagram_callback)),
      bind_callback_(std::move(bind_callback)),
      socket_receive_buffer_bytes_(socket_receive_buffer_bytes),
      retry_delay_(retry_delay) {}

RdbUdpReceiver::~RdbUdpReceiver() { stop(); }

void RdbUdpReceiver::start() {
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&RdbUdpReceiver::run, this);
}

void RdbUdpReceiver::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  close_socket();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool RdbUdpReceiver::bound() const noexcept { return bound_.load(); }

int RdbUdpReceiver::port() const noexcept { return port_; }

int RdbUdpReceiver::bound_port() const noexcept { return bound_port_.load(); }

std::uint64_t RdbUdpReceiver::received_packets() const noexcept {
  return received_packets_.load();
}

std::uint64_t RdbUdpReceiver::received_bytes() const noexcept {
  return received_bytes_.load();
}

std::uint64_t RdbUdpReceiver::bind_errors() const noexcept {
  return bind_errors_.load();
}

double RdbUdpReceiver::seconds_since_last_packet() const noexcept {
  const auto last_packet = last_packet_time_ns_.load();
  if (last_packet == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(steady_time_nanoseconds() - last_packet) / 1.0e9;
}

std::string RdbUdpReceiver::last_sender() const {
  std::lock_guard<std::mutex> lock(sender_mutex_);
  return last_sender_;
}

int RdbUdpReceiver::bind_socket() {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;
  hints.ai_flags = AI_PASSIVE;

  addrinfo *addresses = nullptr;
  const auto port_text = std::to_string(port_);
  const char *host = nullptr;
  if (!bind_address_.empty() && bind_address_ != "*" &&
      bind_address_ != "0.0.0.0") {
    host = bind_address_.c_str();
  }
  if (::getaddrinfo(host, port_text.c_str(), &hints, &addresses) != 0) {
    ++bind_errors_;
    return -1;
  }

  int result_socket = -1;
  for (auto *address = addresses; address != nullptr && running_;
       address = address->ai_next) {
    const int candidate = ::socket(address->ai_family, address->ai_socktype,
                                   address->ai_protocol);
    if (candidate < 0) {
      continue;
    }

    int enabled = 1;
    ::setsockopt(candidate, SOL_SOCKET, SO_REUSEADDR, &enabled,
                 sizeof(enabled));
    if (socket_receive_buffer_bytes_ > 0) {
      ::setsockopt(candidate, SOL_SOCKET, SO_RCVBUF,
                   &socket_receive_buffer_bytes_,
                   sizeof(socket_receive_buffer_bytes_));
    }
    timeval timeout{};
    timeout.tv_usec = kReceiveTimeoutMicroseconds;
    ::setsockopt(candidate, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    if (::bind(candidate, address->ai_addr, address->ai_addrlen) == 0) {
      result_socket = candidate;
      break;
    }
    ::close(candidate);
  }
  ::freeaddrinfo(addresses);

  if (result_socket < 0) {
    ++bind_errors_;
    return -1;
  }

  sockaddr_storage local_address{};
  socklen_t local_address_size = sizeof(local_address);
  if (::getsockname(result_socket, reinterpret_cast<sockaddr *>(&local_address),
                    &local_address_size) == 0) {
    if (local_address.ss_family == AF_INET) {
      const auto *address =
          reinterpret_cast<const sockaddr_in *>(&local_address);
      bound_port_ = static_cast<int>(ntohs(address->sin_port));
    }
  }
  return result_socket;
}

void RdbUdpReceiver::run() {
  std::array<std::uint8_t, kMaximumUdpDatagramSize> receive_buffer{};
  while (running_) {
    const int new_socket = bind_socket();
    if (new_socket < 0) {
      std::this_thread::sleep_for(retry_delay_);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      if (!running_) {
        ::close(new_socket);
        break;
      }
      socket_ = new_socket;
      bound_ = true;
    }
    if (bind_callback_) {
      bind_callback_(true);
    }

    while (running_) {
      sockaddr_storage sender{};
      socklen_t sender_size = sizeof(sender);
      const auto received =
          ::recvfrom(new_socket, receive_buffer.data(), receive_buffer.size(),
                     0, reinterpret_cast<sockaddr *>(&sender), &sender_size);
      if (received < 0 && errno == EINTR) {
        continue;
      }
      if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        continue;
      }
      if (received <= 0) {
        break;
      }

      const auto size = static_cast<std::size_t>(received);
      ++received_packets_;
      received_bytes_ += size;
      last_packet_time_ns_ = steady_time_nanoseconds();
      set_last_sender(&sender, sender_size);
      if (datagram_callback_) {
        datagram_callback_(receive_buffer.data(), size);
      }
    }

    close_socket();
    if (bind_callback_) {
      bind_callback_(false);
    }
    if (running_) {
      std::this_thread::sleep_for(retry_delay_);
    }
  }
}

void RdbUdpReceiver::close_socket() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
  bound_ = false;
  bound_port_ = 0;
}

void RdbUdpReceiver::set_last_sender(const void *address,
                                     const std::size_t address_size) {
  std::array<char, NI_MAXHOST> host{};
  std::array<char, NI_MAXSERV> service{};
  const auto result = ::getnameinfo(
      static_cast<const sockaddr *>(address),
      static_cast<socklen_t>(address_size), host.data(),
      static_cast<socklen_t>(host.size()), service.data(),
      static_cast<socklen_t>(service.size()), NI_NUMERICHOST | NI_NUMERICSERV);
  if (result != 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(sender_mutex_);
  last_sender_ = std::string(host.data()) + ":" + service.data();
}

} // namespace vtd_ros2_bridge
