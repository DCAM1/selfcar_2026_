#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace vtd_ros2_bridge {

constexpr std::size_t kHlvtdLidarHeaderSize = 42U;
constexpr std::uint16_t kHlvtdLidarVersion = 1U;
constexpr std::uint8_t kHlvtdLidarCoordinatesWorld = 1U;
constexpr std::uint8_t kHlvtdLidarCoordinatesSensorRelative = 2U;
constexpr std::uint8_t kHlvtdLidarXyzStride = 12U;

struct HlvtdLidarPoint {
  float x{};
  float y{};
  float z{};
};

struct HlvtdLidarPacketView {
  std::uint16_t version{};
  std::uint32_t frame_id{};
  std::uint64_t timestamp_nanoseconds{};
  double simulation_time{};
  std::uint16_t packet_index{};
  std::uint16_t packet_count{};
  std::uint32_t total_point_count{};
  std::uint32_t point_start_index{};
  std::uint16_t packet_point_count{};
  std::uint8_t coordinate_mode{};
  std::uint8_t point_stride{};
  const std::uint8_t *point_data{};
};

struct HlvtdLidarFrame {
  std::uint32_t frame_id{};
  std::uint64_t timestamp_nanoseconds{};
  double simulation_time{};
  std::uint8_t coordinate_mode{};
  std::vector<HlvtdLidarPoint> points;
};

bool is_hlvtd_lidar_packet(const std::uint8_t *bytes,
                           std::size_t size) noexcept;

bool parse_hlvtd_lidar_packet(const std::uint8_t *bytes, std::size_t size,
                              HlvtdLidarPacketView &packet,
                              std::string *error = nullptr);

class HlvtdLidarFrameAssembler {
public:
  std::optional<HlvtdLidarFrame> add_packet(const HlvtdLidarPacketView &packet,
                                            std::string *error = nullptr);
  void reset();
  std::uint64_t incomplete_frames() const noexcept;

private:
  struct PendingFrame {
    std::uint32_t frame_id{};
    std::uint64_t timestamp_nanoseconds{};
    double simulation_time{};
    std::uint16_t packet_count{};
    std::uint32_t total_point_count{};
    std::uint8_t coordinate_mode{};
    std::vector<HlvtdLidarPoint> points;
    std::vector<std::uint8_t> received_points;
    std::vector<std::uint8_t> received_packets;
    std::size_t received_point_count{};
    std::size_t received_packet_count{};
  };

  std::optional<PendingFrame> pending_;
  std::uint64_t incomplete_frames_{};
};

} // namespace vtd_ros2_bridge
