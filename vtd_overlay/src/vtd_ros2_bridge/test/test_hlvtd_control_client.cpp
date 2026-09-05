#include "vtd_ros2_bridge/hlvtd_control_client.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <limits>
#include <thread>

namespace vtd_ros2_bridge {
namespace {

using namespace std::chrono_literals;

float read_float_le(const std::uint8_t *source) {
  const std::uint32_t bits = static_cast<std::uint32_t>(source[0]) |
                             (static_cast<std::uint32_t>(source[1]) << 8U) |
                             (static_cast<std::uint32_t>(source[2]) << 16U) |
                             (static_cast<std::uint32_t>(source[3]) << 24U);
  float value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void write_u32_le(std::uint8_t *destination, const std::uint32_t value) {
  destination[0] = static_cast<std::uint8_t>(value & 0xffU);
  destination[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  destination[2] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  destination[3] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
}

void write_float_le(std::uint8_t *destination, const float value) {
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  write_u32_le(destination, bits);
}

std::array<std::uint8_t, kHlvtdParticipantDataPacketSize>
participant_packet(const float marker) {
  std::array<std::uint8_t, kHlvtdParticipantDataPacketSize> packet{};
  std::size_t offset = 0U;
  const auto append_float = [&packet, &offset](const float value) {
    write_float_le(packet.data() + offset, value);
    offset += sizeof(float);
  };
  for (const float value : {marker, 2.0F, 3.0F, 0.4F, 0.5F, 0.6F}) {
    append_float(value);
  }
  for (std::size_t index = 0U; index < kHlvtdObjectCount; ++index) {
    write_u32_le(packet.data() + offset,
                 index == 0U ? 42U : static_cast<std::uint32_t>(index + 100U));
    offset += sizeof(std::uint32_t);
    for (std::size_t field = 0U; field < 8U; ++field) {
      append_float(marker + static_cast<float>(index * 10U + field));
    }
  }
  write_u32_le(packet.data() + offset, 77U);
  offset += sizeof(std::int32_t);
  packet[offset++] = 5U;
  EXPECT_EQ(offset, packet.size());
  return packet;
}

int create_bound_server(std::uint16_t &port) {
  const int server = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (server < 0) {
    return -1;
  }
  int reuse = 1;
  ::setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  if (::bind(server, reinterpret_cast<const sockaddr *>(&address),
             sizeof(address)) != 0) {
    ::close(server);
    return -1;
  }
  socklen_t address_size = sizeof(address);
  if (::getsockname(server, reinterpret_cast<sockaddr *>(&address),
                    &address_size) != 0) {
    ::close(server);
    return -1;
  }
  port = ntohs(address.sin_port);
  return server;
}

bool receive_exactly(const int socket, std::uint8_t *data,
                     const std::size_t size) {
  std::size_t received = 0U;
  while (received < size) {
    pollfd descriptor{};
    descriptor.fd = socket;
    descriptor.events = POLLIN;
    if (::poll(&descriptor, 1, 2000) <= 0) {
      return false;
    }
    const auto result = ::recv(socket, data + received, size - received, 0);
    if (result <= 0) {
      return false;
    }
    received += static_cast<std::size_t>(result);
  }
  return true;
}

bool send_all(const int socket, const std::uint8_t *data,
              const std::size_t size) {
  std::size_t sent = 0U;
  while (sent < size) {
    const auto result = ::send(socket, data + sent, size - sent, MSG_NOSIGNAL);
    if (result <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(result);
  }
  return true;
}

TEST(HlvtdControlClient, DecodesDocumentedParticipantPacketLayout) {
  const auto packet = participant_packet(1.25F);
  HlvtdParticipantData data{};
  ASSERT_TRUE(
      decode_hlvtd_participant_data(packet.data(), packet.size(), data));
  EXPECT_FLOAT_EQ(data.ego_x, 1.25F);
  EXPECT_FLOAT_EQ(data.ego_roll, 0.6F);
  EXPECT_EQ(data.objects.front().id, 42U);
  EXPECT_FLOAT_EQ(data.objects.front().x, 1.25F);
  EXPECT_FLOAT_EQ(data.objects.front().height, 8.25F);
  EXPECT_EQ(data.traffic_light.id, 77);
  EXPECT_EQ(data.traffic_light.state, 5U);
  EXPECT_FALSE(
      decode_hlvtd_participant_data(packet.data(), packet.size() - 1U, data));
}

TEST(HlvtdControlClient, ReassemblesPartialParticipantPacket) {
  std::uint16_t port = 0U;
  const int server = create_bound_server(port);
  ASSERT_GE(server, 0);
  ASSERT_EQ(::listen(server, 1), 0);

  std::atomic<bool> received{false};
  HlvtdParticipantData output{};
  HlvtdControlClient client(
      "127.0.0.1", static_cast<int>(port), nullptr, 10ms,
      [&received, &output](const HlvtdParticipantData &data) {
        output = data;
        received = true;
      });
  client.start();

  pollfd descriptor{};
  descriptor.fd = server;
  descriptor.events = POLLIN;
  ASSERT_GT(::poll(&descriptor, 1, 2000), 0);
  const int peer = ::accept(server, nullptr, nullptr);
  ASSERT_GE(peer, 0);

  const auto packet = participant_packet(-3.5F);
  ASSERT_TRUE(send_all(peer, packet.data(), 7U));
  std::this_thread::sleep_for(5ms);
  ASSERT_FALSE(received.load());
  ASSERT_TRUE(send_all(peer, packet.data() + 7U, packet.size() - 7U));
  for (int attempt = 0; attempt < 200 && !received; ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_TRUE(received.load());
  EXPECT_FLOAT_EQ(output.ego_x, -3.5F);
  EXPECT_EQ(output.objects.front().id, 42U);
  EXPECT_EQ(output.traffic_light.id, 77);
  EXPECT_EQ(client.decoded_data_packets(), 1U);

  client.stop();
  ::close(peer);
  ::close(server);
}

TEST(HlvtdControlClient, RetainsOnlyLatestCommandWhileDisconnected) {
  std::uint16_t port = 0U;
  const int server = create_bound_server(port);
  ASSERT_GE(server, 0);

  std::atomic<bool> connected{false};
  HlvtdControlClient client(
      "127.0.0.1", static_cast<int>(port),
      [&connected](const bool value) { connected = value; }, 10ms);
  client.start();

  ASSERT_TRUE(client.send_command(0.1F, 1.0F, 1U));
  ASSERT_TRUE(client.send_command(-0.25F, -1.5F, 2U));
  EXPECT_EQ(client.queued_commands(), 2U);
  EXPECT_EQ(client.overwritten_commands(), 1U);

  ASSERT_EQ(::listen(server, 1), 0);
  pollfd descriptor{};
  descriptor.fd = server;
  descriptor.events = POLLIN;
  ASSERT_GT(::poll(&descriptor, 1, 2000), 0);
  const int peer = ::accept(server, nullptr, nullptr);
  ASSERT_GE(peer, 0);

  std::array<std::uint8_t, 9U> packet{};
  ASSERT_TRUE(receive_exactly(peer, packet.data(), packet.size()));
  EXPECT_FLOAT_EQ(read_float_le(packet.data()), -0.25F);
  EXPECT_FLOAT_EQ(read_float_le(packet.data() + 4U), -1.5F);
  EXPECT_EQ(packet[8], 2U);

  for (int attempt = 0; attempt < 200 && client.sent_commands() != 1U;
       ++attempt) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_TRUE(connected.load());
  EXPECT_EQ(client.sent_commands(), 1U);

  client.stop();
  ::close(peer);
  ::close(server);
}

TEST(HlvtdControlClient, RejectsInvalidCommands) {
  HlvtdControlClient client("127.0.0.1", 9910, nullptr);
  EXPECT_FALSE(client.send_command(0.0F, 0.0F, 0U));

  client.start();
  EXPECT_FALSE(client.send_command(0.0F, 0.0F, 3U));
  EXPECT_FALSE(
      client.send_command(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0U));
  client.stop();
}

} // namespace
} // namespace vtd_ros2_bridge
