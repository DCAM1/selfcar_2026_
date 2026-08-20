#include "vtd_ros2_bridge/rdb_codec.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace vtd_ros2_bridge
{

TEST(RdbCodec, BuildsDriverControlPacket)
{
  RDB_DRIVER_CTRL_t source{};
  source.playerId = 7U;
  source.accelTgt = 1.25F;
  source.steeringTgt = -0.2F;
  source.gear = RDB_GEAR_BOX_POS_D;
  source.validityFlags =
    RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL | RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING;

  const auto packet = make_driver_control_message(12.5, 42U, source);
  int callbacks = 0;
  std::string error;
  ASSERT_TRUE(parse_rdb_message(
      packet.data(), packet.size(),
      [&](const RDB_MSG_HDR_t & message, const RdbEntryView & entry) {
        ++callbacks;
        EXPECT_DOUBLE_EQ(message.simTime, 12.5);
        EXPECT_EQ(message.frameNo, 42U);
        ASSERT_EQ(entry.header->pkgId, RDB_PKG_ID_DRIVER_CTRL);
        ASSERT_EQ(entry.data_size, sizeof(RDB_DRIVER_CTRL_t));
        const auto * control = reinterpret_cast<const RDB_DRIVER_CTRL_t *>(entry.data);
        EXPECT_EQ(control->playerId, 7U);
        EXPECT_FLOAT_EQ(control->accelTgt, 1.25F);
        EXPECT_FLOAT_EQ(control->steeringTgt, -0.2F);
      },
      &error)) << error;
  EXPECT_EQ(callbacks, 1);
}

TEST(RdbCodec, DecodesFragmentedAndConcatenatedStream)
{
  RDB_DRIVER_CTRL_t control{};
  const auto first = make_driver_control_message(1.0, 1U, control);
  const auto second = make_driver_control_message(2.0, 2U, control);
  std::vector<std::uint8_t> stream{0xdeU, 0xadU, 0xbeU};
  stream.insert(stream.end(), first.begin(), first.end());
  stream.insert(stream.end(), second.begin(), second.end());

  RdbStreamDecoder decoder;
  std::vector<std::uint32_t> frames;
  const auto callback = [&](const std::uint8_t * bytes, const std::size_t size) {
      ASSERT_GE(size, sizeof(RDB_MSG_HDR_t));
      frames.push_back(reinterpret_cast<const RDB_MSG_HDR_t *>(bytes)->frameNo);
    };
  decoder.append(stream.data(), 11U, callback);
  decoder.append(stream.data() + 11U, 17U, callback);
  decoder.append(stream.data() + 28U, stream.size() - 28U, callback);

  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0], 1U);
  EXPECT_EQ(frames[1], 2U);
  EXPECT_GT(decoder.rejected_messages(), 0U);
  EXPECT_EQ(decoder.buffered_size(), 0U);
}

TEST(RdbCodec, RejectsTruncatedMessage)
{
  RDB_DRIVER_CTRL_t control{};
  const auto packet = make_driver_control_message(1.0, 1U, control);
  std::string error;
  EXPECT_FALSE(parse_rdb_message(
      packet.data(), packet.size() - 1U,
      [](const RDB_MSG_HDR_t &, const RdbEntryView &) {}, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace vtd_ros2_bridge
