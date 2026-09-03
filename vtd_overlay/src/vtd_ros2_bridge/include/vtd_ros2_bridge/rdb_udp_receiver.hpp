#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace vtd_ros2_bridge {

class RdbUdpReceiver {
public:
  using DatagramCallback =
      std::function<void(const std::uint8_t *, std::size_t)>;
  using BindCallback = std::function<void(bool)>;

  RdbUdpReceiver(
      std::string bind_address, int port, DatagramCallback datagram_callback,
      BindCallback bind_callback,
      int socket_receive_buffer_bytes = 4 * 1024 * 1024,
      std::chrono::milliseconds retry_delay = std::chrono::milliseconds(1000));
  ~RdbUdpReceiver();

  RdbUdpReceiver(const RdbUdpReceiver &) = delete;
  RdbUdpReceiver &operator=(const RdbUdpReceiver &) = delete;

  void start();
  void stop();
  bool bound() const noexcept;
  int port() const noexcept;
  int bound_port() const noexcept;
  std::uint64_t received_packets() const noexcept;
  std::uint64_t received_bytes() const noexcept;
  std::uint64_t bind_errors() const noexcept;
  double seconds_since_last_packet() const noexcept;
  std::string last_sender() const;

private:
  int bind_socket();
  void run();
  void close_socket();
  void set_last_sender(const void *address, std::size_t address_size);

  std::string bind_address_;
  int port_;
  DatagramCallback datagram_callback_;
  BindCallback bind_callback_;
  int socket_receive_buffer_bytes_;
  std::chrono::milliseconds retry_delay_;

  std::atomic<bool> running_{false};
  std::atomic<bool> bound_{false};
  std::atomic<int> bound_port_{0};
  std::atomic<std::uint64_t> received_packets_{0};
  std::atomic<std::uint64_t> received_bytes_{0};
  std::atomic<std::uint64_t> bind_errors_{0};
  std::atomic<std::int64_t> last_packet_time_ns_{0};
  std::thread thread_;
  mutable std::mutex socket_mutex_;
  int socket_{-1};
  mutable std::mutex sender_mutex_;
  std::string last_sender_;
};

} // namespace vtd_ros2_bridge
