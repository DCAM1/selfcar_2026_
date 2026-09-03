#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace vtd_ros2_bridge {

class HlvtdControlClient {
public:
  using ConnectionCallback = std::function<void(bool)>;

  HlvtdControlClient(
      std::string host, int port, ConnectionCallback connection_callback,
      std::chrono::milliseconds reconnect_delay =
          std::chrono::milliseconds(1000));
  ~HlvtdControlClient();

  HlvtdControlClient(const HlvtdControlClient &) = delete;
  HlvtdControlClient &operator=(const HlvtdControlClient &) = delete;

  void start();
  void stop();
  bool send_command(float steering, float target_acceleration,
                    std::uint8_t turn_signal);
  bool connected() const noexcept;
  std::uint64_t received_bytes() const noexcept;

private:
  int connect_socket();
  void run();
  void close_socket();

  std::string host_;
  int port_;
  ConnectionCallback connection_callback_;
  std::chrono::milliseconds reconnect_delay_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<std::uint64_t> received_bytes_{0U};
  std::thread thread_;
  mutable std::mutex socket_mutex_;
  int socket_{-1};
};

} // namespace vtd_ros2_bridge
