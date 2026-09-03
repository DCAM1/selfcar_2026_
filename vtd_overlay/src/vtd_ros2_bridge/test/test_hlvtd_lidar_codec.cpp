#include "vtd_ros2_bridge/hlvtd_lidar_codec.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

namespace vtd_ros2_bridge {
namespace {

void write_u16_le(std::vector<std::uint8_t> &bytes, const std::size_t offset,
                  const std::uint16_t value) {
  bytes[offset] = static_cast<std::uint8_t>(value & 0xffU);
  bytes[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void write_u32_le(std::vector<std::uint8_t> &bytes, const std::size_t offset,
                  const std::uint32_t value) {
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    bytes[offset + index] =
        static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
  }
}

void write_u64_le(std::vector<std::uint8_t> &bytes, const std::size_t offset,
                  const std::uint64_t value) {
  for (std::size_t index = 0U; index < sizeof(value); ++index) {
    bytes[offset + index] =
        static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
  }
}

void write_f32_le(std::vector<std::uint8_t> &bytes, const std::size_t offset,
                  const float value) {
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  write_u32_le(bytes, offset, bits);
}

void write_f64_le(std::vector<std::uint8_t> &bytes, const std::size_t offset,
                  const double value) {
  std::uint64_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  write_u64_le(bytes, offset, bits);
}

std::vector<std::uint8_t>
make_packet(const std::uint32_t frame_id, const std::uint16_t packet_index,
            const std::uint16_t packet_count, const std::uint32_t total_points,
            const std::uint32_t point_start,
            const std::vector<HlvtdLidarPoint> &points) {
  std::vector<std::uint8_t> bytes(
      kHlvtdLidarHeaderSize + points.size() * kHlvtdLidarXyzStride, 0U);
  bytes[0] = 'I';
  bytes[1] = 'V';
  bytes[2] = 'H';
  bytes[3] = 'L';
  write_u16_le(bytes, 4U, kHlvtdLidarVersion);
  write_u32_le(bytes, 6U, frame_id);
  write_u64_le(bytes, 10U, 123456789U);
  write_f64_le(bytes, 18U, 42.25);
  write_u16_le(bytes, 26U, packet_index);
  write_u16_le(bytes, 28U, packet_count);
  write_u32_le(bytes, 30U, total_points);
  write_u32_le(bytes, 34U, point_start);
  write_u16_le(bytes, 38U, static_cast<std::uint16_t>(points.size()));
  bytes[40U] = kHlvtdLidarCoordinatesWorld;
  bytes[41U] = kHlvtdLidarXyzStride;
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const auto offset = kHlvtdLidarHeaderSize + index * kHlvtdLidarXyzStride;
    write_f32_le(bytes, offset, points[index].x);
    write_f32_le(bytes, offset + 4U, points[index].y);
    write_f32_le(bytes, offset + 8U, points[index].z);
  }
  return bytes;
}

TEST(HlvtdLidarCodec, ParsesSenderPacketLayout) {
  const auto bytes = make_packet(77U, 0U, 1U, 2U, 0U,
                                 {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}});
  HlvtdLidarPacketView packet;
  std::string error;

  ASSERT_TRUE(
      parse_hlvtd_lidar_packet(bytes.data(), bytes.size(), packet, &error))
      << error;
  EXPECT_EQ(packet.frame_id, 77U);
  EXPECT_EQ(packet.timestamp_nanoseconds, 123456789U);
  EXPECT_DOUBLE_EQ(packet.simulation_time, 42.25);
  EXPECT_EQ(packet.packet_point_count, 2U);
  EXPECT_EQ(packet.coordinate_mode, kHlvtdLidarCoordinatesWorld);
  EXPECT_EQ(packet.point_stride, kHlvtdLidarXyzStride);
}

TEST(HlvtdLidarCodec, RejectsWrongMagicAndSize) {
  auto bytes = make_packet(1U, 0U, 1U, 1U, 0U, {{1.0F, 2.0F, 3.0F}});
  HlvtdLidarPacketView packet;
  std::string error;

  bytes[1] = 'X';
  EXPECT_FALSE(
      parse_hlvtd_lidar_packet(bytes.data(), bytes.size(), packet, &error));
  bytes[1] = 'V';
  bytes.pop_back();
  EXPECT_FALSE(
      parse_hlvtd_lidar_packet(bytes.data(), bytes.size(), packet, &error));
}

TEST(HlvtdLidarCodec, ReassemblesOutOfOrderPackets) {
  const auto first =
      make_packet(9U, 0U, 2U, 3U, 0U, {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}});
  const auto second = make_packet(9U, 1U, 2U, 3U, 2U, {{7.0F, 8.0F, 9.0F}});
  HlvtdLidarPacketView first_view;
  HlvtdLidarPacketView second_view;
  ASSERT_TRUE(parse_hlvtd_lidar_packet(first.data(), first.size(), first_view));
  ASSERT_TRUE(
      parse_hlvtd_lidar_packet(second.data(), second.size(), second_view));

  HlvtdLidarFrameAssembler assembler;
  EXPECT_FALSE(assembler.add_packet(second_view));
  const auto frame = assembler.add_packet(first_view);
  ASSERT_TRUE(frame);
  ASSERT_EQ(frame->points.size(), 3U);
  EXPECT_FLOAT_EQ(frame->points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(frame->points[1].y, 5.0F);
  EXPECT_FLOAT_EQ(frame->points[2].z, 9.0F);
  EXPECT_EQ(assembler.incomplete_frames(), 0U);
}

TEST(HlvtdLidarCodec, CountsFrameReplacedBeforeCompletion) {
  const auto old_packet = make_packet(9U, 0U, 2U, 2U, 0U, {{1.0F, 2.0F, 3.0F}});
  const auto new_packet =
      make_packet(10U, 0U, 1U, 1U, 0U, {{4.0F, 5.0F, 6.0F}});
  HlvtdLidarPacketView old_view;
  HlvtdLidarPacketView new_view;
  ASSERT_TRUE(
      parse_hlvtd_lidar_packet(old_packet.data(), old_packet.size(), old_view));
  ASSERT_TRUE(
      parse_hlvtd_lidar_packet(new_packet.data(), new_packet.size(), new_view));

  HlvtdLidarFrameAssembler assembler;
  EXPECT_FALSE(assembler.add_packet(old_view));
  EXPECT_TRUE(assembler.add_packet(new_view));
  EXPECT_EQ(assembler.incomplete_frames(), 1U);
}

} // namespace
} // namespace vtd_ros2_bridge
