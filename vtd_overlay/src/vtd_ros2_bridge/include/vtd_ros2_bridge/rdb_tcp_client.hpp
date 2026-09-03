#pragma once

#include "vtd_ros2_bridge/rdb_codec.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace vtd_ros2_bridge
{

class RdbTcpClient
{
public:
  using MessageCallback = std::function<void(const std::uint8_t *, std::size_t)>;
  using ConnectionCallback = std::function<void(bool)>;

  RdbTcpClient(
    std::string label, std::string host, int port, MessageCallback message_callback,
    ConnectionCallback connection_callback,
    std::chrono::milliseconds reconnect_delay = std::chrono::milliseconds(1000));
  ~RdbTcpClient();

  RdbTcpClient(const RdbTcpClient &) = delete;
  RdbTcpClient & operator=(const RdbTcpClient &) = delete;

  void start();
  void stop();
  bool send_bytes(const std::vector<std::uint8_t> & bytes);
  bool connected() const noexcept;
  int port() const noexcept;
  std::uint64_t received_messages() const noexcept;

private:
  int connect_socket();
  void run();
  void close_socket();

  std::string label_;
  std::string host_;
  int port_;
  MessageCallback message_callback_;
  ConnectionCallback connection_callback_;
  std::chrono::milliseconds reconnect_delay_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<std::uint64_t> received_messages_{0};
  std::thread thread_;
  mutable std::mutex socket_mutex_;
  int socket_{-1};
  RdbStreamDecoder decoder_;
};

}  // namespace vtd_ros2_bridge
