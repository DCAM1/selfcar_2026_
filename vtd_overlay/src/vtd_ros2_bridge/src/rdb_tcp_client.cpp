#include "vtd_ros2_bridge/rdb_tcp_client.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>

namespace vtd_ros2_bridge
{

RdbTcpClient::RdbTcpClient(
  std::string label, std::string host, const int port, MessageCallback message_callback,
  ConnectionCallback connection_callback, const std::chrono::milliseconds reconnect_delay,
  const bool decode_rdb_stream)
: label_(std::move(label)),
  host_(std::move(host)),
  port_(port),
  message_callback_(std::move(message_callback)),
  connection_callback_(std::move(connection_callback)),
  reconnect_delay_(reconnect_delay),
  decode_rdb_stream_(decode_rdb_stream)
{
}

RdbTcpClient::~RdbTcpClient()
{
  stop();
}

void RdbTcpClient::start()
{
  if (port_ <= 0 || running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&RdbTcpClient::run, this);
}

void RdbTcpClient::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  close_socket();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool RdbTcpClient::send_bytes(const std::vector<std::uint8_t> & bytes)
{
  if (bytes.empty()) {
    return true;
  }
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_ < 0 || !connected_) {
    return false;
  }

  std::size_t sent = 0U;
  while (sent < bytes.size()) {
    const auto result = ::send(
      socket_, bytes.data() + sent, bytes.size() - sent, MSG_NOSIGNAL);
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

bool RdbTcpClient::connected() const noexcept
{
  return connected_.load();
}

int RdbTcpClient::port() const noexcept
{
  return port_;
}

std::uint64_t RdbTcpClient::received_messages() const noexcept
{
  return received_messages_.load();
}

int RdbTcpClient::connect_socket()
{
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  addrinfo * addresses = nullptr;
  const auto port_text = std::to_string(port_);
  if (::getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &addresses) != 0) {
    return -1;
  }

  int result_socket = -1;
  for (auto * address = addresses; address != nullptr && running_; address = address->ai_next) {
    const int candidate = ::socket(address->ai_family, address->ai_socktype, address->ai_protocol);
    if (candidate < 0) {
      continue;
    }
    int enabled = 1;
    ::setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    if (::connect(candidate, address->ai_addr, address->ai_addrlen) == 0) {
      result_socket = candidate;
      break;
    }
    ::close(candidate);
  }
  ::freeaddrinfo(addresses);
  return result_socket;
}

void RdbTcpClient::run()
{
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
    decoder_.clear();
    if (connection_callback_) {
      connection_callback_(true);
    }

    while (running_) {
      const auto received = ::recv(new_socket, receive_buffer.data(), receive_buffer.size(), 0);
      if (received < 0 && errno == EINTR) {
        continue;
      }
      if (received <= 0) {
        break;
      }
      if (decode_rdb_stream_) {
        decoder_.append(
          receive_buffer.data(), static_cast<std::size_t>(received),
          [this](const std::uint8_t * message, const std::size_t size) {
            ++received_messages_;
            if (message_callback_) {
              message_callback_(message, size);
            }
          });
      } else {
        // The HL_VTD participant socket sends a fixed-size, headerless state
        // stream. Drain it so the server never blocks; state still comes from
        // the full RDB channel handled by the other client.
        ++received_messages_;
      }
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

void RdbTcpClient::close_socket()
{
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_ >= 0) {
    ::shutdown(socket_, SHUT_RDWR);
    ::close(socket_);
    socket_ = -1;
  }
  connected_ = false;
}

}  // namespace vtd_ros2_bridge
