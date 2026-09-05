#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace vtd_ros2_bridge {

constexpr std::size_t kHlvtdObjectCount = 30U;
constexpr std::size_t kHlvtdParticipantDataPacketSize = 1109U;

struct HlvtdParticipantObject {
  std::uint32_t id{};
  float x{};
  float y{};
  float z{};
  float heading{};
  float speed{};
  float length{};
  float width{};
  float height{};
};

struct HlvtdParticipantTrafficLight {
  std::int32_t id{};
  std::uint8_t state{};
};

struct HlvtdParticipantData {
  float ego_x{};
  float ego_y{};
  float ego_z{};
  float ego_heading{};
  float ego_pitch{};
  float ego_roll{};
  std::array<HlvtdParticipantObject, kHlvtdObjectCount> objects{};
  HlvtdParticipantTrafficLight traffic_light{};
};

// Decode the fixed, headerless VTD -> participant record defined by
// Downloads/인터페이스 API.xlsx. All multibyte fields are little-endian.
bool decode_hlvtd_participant_data(const std::uint8_t *packet,
                                   std::size_t packet_size,
                                   HlvtdParticipantData &output);

class HlvtdControlClient {
public:
  using ConnectionCallback = std::function<void(bool)>;
  using DataCallback = std::function<void(const HlvtdParticipantData &)>;

  HlvtdControlClient(std::string host, int port,
                     ConnectionCallback connection_callback,
                     std::chrono::milliseconds reconnect_delay =
                         std::chrono::milliseconds(1000),
                     DataCallback data_callback = nullptr);
  ~HlvtdControlClient();

  HlvtdControlClient(const HlvtdControlClient &) = delete;
  HlvtdControlClient &operator=(const HlvtdControlClient &) = delete;

  void start();
  void stop();
  bool send_command(float steering, float target_acceleration,
                    std::uint8_t turn_signal);
  bool connected() const noexcept;
  std::uint64_t received_bytes() const noexcept;
  std::uint64_t decoded_data_packets() const noexcept;
  std::uint64_t skipped_data_packets() const noexcept;
  std::uint64_t queued_commands() const noexcept;
  std::uint64_t sent_commands() const noexcept;
  std::uint64_t overwritten_commands() const noexcept;

private:
  static constexpr std::size_t kControlPacketSize = 9U;
  using ControlPacket = std::array<std::uint8_t, kControlPacketSize>;

  int connect_socket();
  void receive_loop();
  void transmit_loop();
  bool send_packet(const ControlPacket &packet);
  void clear_pending_command();
  void close_socket();

  std::string host_;
  int port_;
  ConnectionCallback connection_callback_;
  std::chrono::milliseconds reconnect_delay_;
  DataCallback data_callback_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::atomic<std::uint64_t> received_bytes_{0U};
  std::atomic<std::uint64_t> decoded_data_packets_{0U};
  std::atomic<std::uint64_t> skipped_data_packets_{0U};
  std::atomic<std::uint64_t> queued_commands_{0U};
  std::atomic<std::uint64_t> sent_commands_{0U};
  std::atomic<std::uint64_t> overwritten_commands_{0U};
  std::thread receive_thread_;
  std::thread transmit_thread_;
  mutable std::mutex socket_mutex_;
  int socket_{-1};
  std::mutex pending_mutex_;
  std::condition_variable pending_cv_;
  ControlPacket pending_command_{};
  bool pending_command_valid_{false};
};

} // namespace vtd_ros2_bridge
