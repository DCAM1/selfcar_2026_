#include "vtd_ros2_bridge/hlvtd_control_client.hpp"

#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <utility>

namespace vtd_ros2_bridge {
namespace {

constexpr std::size_t kControlPacketSize = 9U;

void write_float_le(std::uint8_t *destination, const float value) {
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  destination[0] = static_cast<std::uint8_t>(bits & 0xffU);
  destination[1] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
  destination[2] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
  destination[3] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
}

} // namespace

HlvtdControlClient::HlvtdControlClient(
    std::string host, const int port,
    ConnectionCallback connection_callback,
    const std::chrono::milliseconds reconnect_delay)
    : host_(std::move(host)), port_(port),
      connection_callback_(std::move(connection_callback)),
      reconnect_delay_(reconnect_delay) {}

HlvtdControlClient::~HlvtdControlClient() { stop(); }

void HlvtdControlClient::start() {
  if (port_ <= 0 || running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&HlvtdControlClient::run, this);
}

void HlvtdControlClient::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  close_socket();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool HlvtdControlClient::send_command(const float steering,
                                      const float target_acceleration,
                                      const std::uint8_t turn_signal) {
  if (!std::isfinite(steering) || !std::isfinite(target_acceleration) ||
      turn_signal > 2U) {
    return false;
  }

  std::array<std::uint8_t, kControlPacketSize> packet{};
  write_float_le(packet.data(), steering);
  write_float_le(packet.data() + sizeof(float), target_acceleration);
  packet[8] = turn_signal;

  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_ < 0 || !connected_) {
    return false;
  }
  std::size_t sent = 0U;
  while (sent < packet.size()) {
    const auto result = ::send(socket_, packet.data() + sent,
                               packet.size() - sent, MSG_NOSIGNAL);
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(result);
  }
  return true;
}

bool HlvtdControlClient::connected() const noexcept {
  return connected_.load();
}

std::uint64_t HlvtdControlClient::received_bytes() const noexcept {
  return received_bytes_.load();
}

int HlvtdControlClient::connect_socket() {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  addrinfo *addresses = nullptr;
  const auto port_text = std::to_string(port_);
  if (::getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &addresses) != 0) {
    return -1;
  }

  int result_socket = -1;
  for (auto *address = addresses; address != nullptr && running_;
       address = address->ai_next) {
    const int candidate =
        ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (candidate < 0) {
      continue;
    }
    int enabled = 1;
    ::setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY, &enabled,
                 sizeof(enabled));
    ::setsockopt(candidate, SOL_SOCKET, SO_KEEPALIVE, &enabled,
                 sizeof(enabled));
    if (::connect(candidate, address->ai_addr, address->ai_addrlen) == 0) {
      result_socket = candidate;
      break;
    }
    ::close(candidate);
  }
  ::freeaddrinfo(addresses);
  return result_socket;
}

void HlvtdControlClient::run() {
  std::array<std::uint8_t, 256U * 1024U> receive_buffer{};
  while (running_) {
    const int new_socket = connect_socket();
    if (new_socket < 0) {
      std::this_thread::sleep_for(reconnect_delay_);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      socket_ = new_socket;
      connected_ = true;
    }
    if (connection_callback_) {
      connection_callback_(true);
    }

    while (running_) {
      const auto received =
          ::recv(new_socket, receive_buffer.data(), receive_buffer.size(), 0);
      if (received < 0 && errno == EINTR) {
        continue;
      }
      if (received <= 0) {
        break;
      }
      received_bytes_.fetch_add(static_cast<std::uint64_t>(received));
    }

    close_socket();
    if (connection_callback_) {
      connection_callback_(false);
    }
    if (running_) {
      std::this_thread::sleep_for(reconnect_delay_);
    }
  }
}

void HlvtdControlClient::close_socket() {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_ >= 0) {
    ::shutdown(socket_, SHUT_RDWR);
    ::close(socket_);
    socket_ = -1;
  }
  connected_ = false;
}

} // namespace vtd_ros2_bridge
