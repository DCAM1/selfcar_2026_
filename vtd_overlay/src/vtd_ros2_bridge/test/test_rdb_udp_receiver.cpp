#include "vtd_ros2_bridge/rdb_codec.hpp"
#include "vtd_ros2_bridge/rdb_udp_receiver.hpp"

#include <VtdToolkit/viRDBIcd.h>

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <thread>
#include <vector>

namespace vtd_ros2_bridge {

TEST(RdbUdpReceiver, ReceivesCompleteRdbDatagram) {
  std::promise<std::vector<std::uint8_t>> received_promise;
  auto received_future = received_promise.get_future();
  std::atomic<bool> callback_completed{false};
  RdbUdpReceiver receiver(
      "127.0.0.1", 0,
      [&](const std::uint8_t *bytes, const std::size_t size) {
        if (!callback_completed.exchange(true)) {
          received_promise.set_value(
              std::vector<std::uint8_t>(bytes, bytes + size));
        }
      },
      [](bool) {});
  receiver.start();

  const auto bind_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!receiver.bound() &&
         std::chrono::steady_clock::now() < bind_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(receiver.bound());
  ASSERT_GT(receiver.bound_port(), 0);

  RDB_DRIVER_CTRL_t control{};
  const auto packet = make_driver_control_message(3.0, 17U, control);
  const int socket_fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  ASSERT_GE(socket_fd, 0);
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port =
      htons(static_cast<std::uint16_t>(receiver.bound_port()));
  ASSERT_EQ(::inet_pton(AF_INET, "127.0.0.1", &destination.sin_addr), 1);
  ASSERT_EQ(::sendto(socket_fd, packet.data(), packet.size(), 0,
                     reinterpret_cast<const sockaddr *>(&destination),
                     sizeof(destination)),
            static_cast<ssize_t>(packet.size()));
  ::close(socket_fd);

  ASSERT_EQ(received_future.wait_for(std::chrono::seconds(2)),
            std::future_status::ready);
  EXPECT_EQ(received_future.get(), packet);
  EXPECT_EQ(receiver.received_packets(), 1U);
  EXPECT_EQ(receiver.received_bytes(), packet.size());
  EXPECT_FALSE(receiver.last_sender().empty());
  EXPECT_LT(receiver.seconds_since_last_packet(), 2.0);
  receiver.stop();
  EXPECT_FALSE(receiver.bound());
}

} // namespace vtd_ros2_bridge
