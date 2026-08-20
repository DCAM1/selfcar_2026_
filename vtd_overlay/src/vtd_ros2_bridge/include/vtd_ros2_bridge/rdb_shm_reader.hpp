#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

namespace vtd_ros2_bridge
{

class RdbShmReader
{
public:
  using MessageCallback = std::function<void(const std::uint8_t *, std::size_t)>;
  using ConnectionCallback = std::function<void(bool)>;

  RdbShmReader(
    int key, std::uint32_t check_mask, MessageCallback message_callback,
    ConnectionCallback connection_callback,
    std::chrono::milliseconds poll_interval = std::chrono::milliseconds(2));
  ~RdbShmReader();

  RdbShmReader(const RdbShmReader &) = delete;
  RdbShmReader & operator=(const RdbShmReader &) = delete;

  void start();
  void stop();
  bool connected() const noexcept;
  std::uint64_t received_messages() const noexcept;

private:
  bool attach();
  void detach();
  bool read_once();
  void run();

  int key_;
  std::uint32_t check_mask_;
  MessageCallback message_callback_;
  ConnectionCallback connection_callback_;
  std::chrono::milliseconds poll_interval_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<std::uint64_t> received_messages_{0U};
  std::thread thread_;
  int shm_id_{-1};
  std::uint8_t * shm_{nullptr};
  std::size_t shm_size_{0U};
  std::uint32_t last_frame_{0U};
};

}  // namespace vtd_ros2_bridge
