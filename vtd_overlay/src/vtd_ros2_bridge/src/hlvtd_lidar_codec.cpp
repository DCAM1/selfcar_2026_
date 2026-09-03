#include "vtd_ros2_bridge/hlvtd_lidar_codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <utility>

namespace vtd_ros2_bridge {

namespace {

constexpr std::array<std::uint8_t, 4U> kMagic{'I', 'V', 'H', 'L'};
constexpr std::uint32_t kMaximumPointCount = 10U * 1000U * 1000U;

void set_error(std::string *error, const std::string &value) {
  if (error) {
    *error = value;
  }
}

std::uint16_t read_u16_le(const std::uint8_t *bytes) {
  return static_cast<std::uint16_t>(bytes[0]) |
         (static_cast<std::uint16_t>(bytes[1]) << 8U);
}

std::uint32_t read_u32_le(const std::uint8_t *bytes) {
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

std::uint64_t read_u64_le(const std::uint8_t *bytes) {
  std::uint64_t value = 0U;
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    value |= static_cast<std::uint64_t>(bytes[index]) << (index * 8U);
  }
  return value;
}

float read_f32_le(const std::uint8_t *bytes) {
  const std::uint32_t bits = read_u32_le(bytes);
  float value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

double read_f64_le(const std::uint8_t *bytes) {
  const std::uint64_t bits = read_u64_le(bytes);
  double value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

} // namespace

bool is_hlvtd_lidar_packet(const std::uint8_t *bytes,
                           const std::size_t size) noexcept {
  return bytes != nullptr && size >= kMagic.size() &&
         std::equal(kMagic.begin(), kMagic.end(), bytes);
}

bool parse_hlvtd_lidar_packet(const std::uint8_t *bytes, const std::size_t size,
                              HlvtdLidarPacketView &packet,
                              std::string *error) {
  if (!bytes || size < kHlvtdLidarHeaderSize) {
    set_error(error, "packet is smaller than the 42-byte IVHL header");
    return false;
  }
  if (!is_hlvtd_lidar_packet(bytes, size)) {
    set_error(error, "invalid IVHL magic");
    return false;
  }

  HlvtdLidarPacketView parsed;
  parsed.version = read_u16_le(bytes + 4U);
  parsed.frame_id = read_u32_le(bytes + 6U);
  parsed.timestamp_nanoseconds = read_u64_le(bytes + 10U);
  parsed.simulation_time = read_f64_le(bytes + 18U);
  parsed.packet_index = read_u16_le(bytes + 26U);
  parsed.packet_count = read_u16_le(bytes + 28U);
  parsed.total_point_count = read_u32_le(bytes + 30U);
  parsed.point_start_index = read_u32_le(bytes + 34U);
  parsed.packet_point_count = read_u16_le(bytes + 38U);
  parsed.coordinate_mode = bytes[40U];
  parsed.point_stride = bytes[41U];
  parsed.point_data = bytes + kHlvtdLidarHeaderSize;

  if (parsed.version != kHlvtdLidarVersion) {
    set_error(error, "unsupported IVHL version");
    return false;
  }
  if (!std::isfinite(parsed.simulation_time) || parsed.simulation_time < 0.0) {
    set_error(error, "invalid IVHL simulation time");
    return false;
  }
  if (parsed.packet_count == 0U || parsed.packet_index >= parsed.packet_count) {
    set_error(error, "invalid IVHL packet index/count");
    return false;
  }
  if (parsed.total_point_count == 0U ||
      parsed.total_point_count > kMaximumPointCount) {
    set_error(error, "invalid IVHL total point count");
    return false;
  }
  if (parsed.coordinate_mode != kHlvtdLidarCoordinatesWorld &&
      parsed.coordinate_mode != kHlvtdLidarCoordinatesSensorRelative) {
    set_error(error, "unsupported IVHL coordinate mode");
    return false;
  }
  if (parsed.point_stride < kHlvtdLidarXyzStride) {
    set_error(error, "IVHL point stride is smaller than float32 XYZ");
    return false;
  }
  const auto point_end = static_cast<std::uint64_t>(parsed.point_start_index) +
                         parsed.packet_point_count;
  if (parsed.packet_point_count == 0U || point_end > parsed.total_point_count) {
    set_error(error, "IVHL point range exceeds the frame");
    return false;
  }
  const auto payload_size =
      static_cast<std::uint64_t>(parsed.packet_point_count) *
      parsed.point_stride;
  const auto expected_size =
      static_cast<std::uint64_t>(kHlvtdLidarHeaderSize) + payload_size;
  if (expected_size != size) {
    set_error(error, "IVHL datagram size does not match point count/stride");
    return false;
  }

  packet = parsed;
  if (error) {
    error->clear();
  }
  return true;
}

std::optional<HlvtdLidarFrame>
HlvtdLidarFrameAssembler::add_packet(const HlvtdLidarPacketView &packet,
                                     std::string *error) {
  if (error) {
    error->clear();
  }
  if (!pending_ || pending_->frame_id != packet.frame_id) {
    if (pending_) {
      ++incomplete_frames_;
    }
    PendingFrame frame;
    frame.frame_id = packet.frame_id;
    frame.timestamp_nanoseconds = packet.timestamp_nanoseconds;
    frame.simulation_time = packet.simulation_time;
    frame.packet_count = packet.packet_count;
    frame.total_point_count = packet.total_point_count;
    frame.coordinate_mode = packet.coordinate_mode;
    frame.points.resize(packet.total_point_count);
    frame.received_points.resize(packet.total_point_count, 0U);
    frame.received_packets.resize(packet.packet_count, 0U);
    pending_ = std::move(frame);
  }

  auto &frame = *pending_;
  if (frame.packet_count != packet.packet_count ||
      frame.total_point_count != packet.total_point_count ||
      frame.coordinate_mode != packet.coordinate_mode ||
      frame.timestamp_nanoseconds != packet.timestamp_nanoseconds ||
      frame.simulation_time != packet.simulation_time) {
    set_error(error, "IVHL packet metadata changed within a frame");
    return std::nullopt;
  }
  if (frame.received_packets[packet.packet_index] != 0U) {
    return std::nullopt;
  }

  const auto start = static_cast<std::size_t>(packet.point_start_index);
  const auto count = static_cast<std::size_t>(packet.packet_point_count);
  for (std::size_t index = 0U; index < count; ++index) {
    if (frame.received_points[start + index] != 0U) {
      set_error(error, "IVHL packet point ranges overlap");
      return std::nullopt;
    }
  }
  for (std::size_t index = 0U; index < count; ++index) {
    const auto *point = packet.point_data + index * packet.point_stride;
    frame.points[start + index] = HlvtdLidarPoint{
        read_f32_le(point), read_f32_le(point + 4U), read_f32_le(point + 8U)};
    frame.received_points[start + index] = 1U;
  }
  frame.received_packets[packet.packet_index] = 1U;
  frame.received_point_count += count;
  ++frame.received_packet_count;

  if (frame.received_packet_count != frame.packet_count ||
      frame.received_point_count != frame.total_point_count) {
    return std::nullopt;
  }

  HlvtdLidarFrame complete;
  complete.frame_id = frame.frame_id;
  complete.timestamp_nanoseconds = frame.timestamp_nanoseconds;
  complete.simulation_time = frame.simulation_time;
  complete.coordinate_mode = frame.coordinate_mode;
  complete.points = std::move(frame.points);
  pending_.reset();
  return complete;
}

void HlvtdLidarFrameAssembler::reset() {
  if (pending_) {
    ++incomplete_frames_;
    pending_.reset();
  }
}

std::uint64_t HlvtdLidarFrameAssembler::incomplete_frames() const noexcept {
  return incomplete_frames_;
}

} // namespace vtd_ros2_bridge
