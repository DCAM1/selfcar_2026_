#include "vtd_ros2_bridge/hlvtd_control_client.hpp"

#include <netdb.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <optional>
#include <utility>

namespace vtd_ros2_bridge {
namespace {

void write_float_le(std::uint8_t *destination, const float value) {
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  std::uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  destination[0] = static_cast<std::uint8_t>(bits & 0xffU);
  destination[1] = static_cast<std::uint8_t>((bits >> 8U) & 0xffU);
  destination[2] = static_cast<std::uint8_t>((bits >> 16U) & 0xffU);
  destination[3] = static_cast<std::uint8_t>((bits >> 24U) & 0xffU);
}

std::uint32_t read_u32_le(const std::uint8_t *source) {
  return static_cast<std::uint32_t>(source[0]) |
         (static_cast<std::uint32_t>(source[1]) << 8U) |
         (static_cast<std::uint32_t>(source[2]) << 16U) |
         (static_cast<std::uint32_t>(source[3]) << 24U);
}

std::int32_t read_i32_le(const std::uint8_t *source) {
  const auto bits = read_u32_le(source);
  std::int32_t value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

float read_float_le(const std::uint8_t *source) {
  const auto bits = read_u32_le(source);
  float value{};
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

} // namespace

bool decode_hlvtd_participant_data(const std::uint8_t *packet,
                                   const std::size_t packet_size,
                                   HlvtdParticipantData &output) {
  if (packet == nullptr || packet_size != kHlvtdParticipantDataPacketSize) {
    return false;
  }

  std::size_t offset = 0U;
  const auto next_float = [&packet, &offset]() {
    const float value = read_float_le(packet + offset);
    offset += sizeof(float);
    return value;
  };
  output.ego_x = next_float();
  output.ego_y = next_float();
  output.ego_z = next_float();
  output.ego_heading = next_float();
  output.ego_pitch = next_float();
  output.ego_roll = next_float();

  for (auto &object : output.objects) {
    object.id = read_u32_le(packet + offset);
    offset += sizeof(std::uint32_t);
    object.x = next_float();
    object.y = next_float();
    object.z = next_float();
    object.heading = next_float();
    object.speed = next_float();
    object.length = next_float();
    object.width = next_float();
    object.height = next_float();
  }

  output.traffic_light.id = read_i32_le(packet + offset);
  offset += sizeof(std::int32_t);
  output.traffic_light.state = packet[offset++];
  return offset == packet_size;
}

HlvtdControlClient::HlvtdControlClient(
    std::string host, const int port, ConnectionCallback connection_callback,
    const std::chrono::milliseconds reconnect_delay, DataCallback data_callback)
    : host_(std::move(host)), port_(port),
      connection_callback_(std::move(connection_callback)),
      reconnect_delay_(reconnect_delay),
      data_callback_(std::move(data_callback)) {}

HlvtdControlClient::~HlvtdControlClient() { stop(); }

void HlvtdControlClient::start() {
  if (port_ <= 0 || running_.exchange(true)) {
    return;
  }
  receive_thread_ = std::thread(&HlvtdControlClient::receive_loop, this);
  transmit_thread_ = std::thread(&HlvtdControlClient::transmit_loop, this);
}

void HlvtdControlClient::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  pending_cv_.notify_all();
  close_socket();
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (transmit_thread_.joinable()) {
    transmit_thread_.join();
  }
}

bool HlvtdControlClient::send_command(const float steering,
                                      const float target_acceleration,
                                      const std::uint8_t turn_signal) {
  if (!std::isfinite(steering) || !std::isfinite(target_acceleration) ||
      turn_signal > 2U) {
    return false;
  }

  ControlPacket packet{};
  write_float_le(packet.data(), steering);
  write_float_le(packet.data() + sizeof(float), target_acceleration);
  packet[8] = turn_signal;

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (!running_) {
      return false;
    }
    if (pending_command_valid_) {
      ++overwritten_commands_;
    }
    pending_command_ = packet;
    pending_command_valid_ = true;
    ++queued_commands_;
  }
  pending_cv_.notify_one();
  return true;
}

bool HlvtdControlClient::connected() const noexcept {
  return connected_.load();
}

std::uint64_t HlvtdControlClient::received_bytes() const noexcept {
  return received_bytes_.load();
}

std::uint64_t HlvtdControlClient::decoded_data_packets() const noexcept {
  return decoded_data_packets_.load();
}

std::uint64_t HlvtdControlClient::skipped_data_packets() const noexcept {
  return skipped_data_packets_.load();
}

std::uint64_t HlvtdControlClient::queued_commands() const noexcept {
  return queued_commands_.load();
}

std::uint64_t HlvtdControlClient::sent_commands() const noexcept {
  return sent_commands_.load();
}

std::uint64_t HlvtdControlClient::overwritten_commands() const noexcept {
  return overwritten_commands_.load();
}

int HlvtdControlClient::connect_socket() {
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  addrinfo *addresses = nullptr;
  const auto port_text = std::to_string(port_);
  if (::getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &addresses) !=
      0) {
    return -1;
  }

  int result_socket = -1;
  for (auto *address = addresses; address != nullptr && running_;
       address = address->ai_next) {
    const int candidate = ::socket(address->ai_family, address->ai_socktype,
                                   address->ai_protocol);
    if (candidate < 0) {
      continue;
    }
    int enabled = 1;
    ::setsockopt(candidate, IPPROTO_TCP, TCP_NODELAY, &enabled,
                 sizeof(enabled));
    ::setsockopt(candidate, SOL_SOCKET, SO_KEEPALIVE, &enabled,
                 sizeof(enabled));
    if (::connect(candidate, address->ai_addr, address->ai_addrlen) == 0) {
      result_socket = candidate;
      break;
    }
    ::close(candidate);
  }
  ::freeaddrinfo(addresses);
  return result_socket;
}

void HlvtdControlClient::receive_loop() {
  std::array<std::uint8_t, 256U * 1024U> receive_buffer{};
  while (running_) {
    const int new_socket = connect_socket();
    if (new_socket < 0) {
      std::this_thread::sleep_for(reconnect_delay_);
      continue;
    }
    {
      std::lock_guard<std::mutex> lock(socket_mutex_);
      if (!running_) {
        ::close(new_socket);
        break;
      }
      socket_ = new_socket;
      connected_ = true;
    }
    pending_cv_.notify_one();
    if (connection_callback_) {
      connection_callback_(true);
    }

    std::array<std::uint8_t, kHlvtdParticipantDataPacketSize> packet_buffer{};
    std::size_t packet_bytes = 0U;
    while (running_) {
      auto received =
          ::recv(new_socket, receive_buffer.data(), receive_buffer.size(), 0);
      if (received < 0 && errno == EINTR) {
        continue;
      }
      if (received <= 0) {
        break;
      }

      std::optional<HlvtdParticipantData> newest_data;
      std::size_t decoded_in_batch = 0U;
      bool connection_lost = false;
      const auto consume = [&](const std::uint8_t *bytes,
                               const std::size_t byte_count) {
        std::size_t consumed = 0U;
        while (consumed < byte_count) {
          const auto copied = std::min(packet_buffer.size() - packet_bytes,
                                       byte_count - consumed);
          std::memcpy(packet_buffer.data() + packet_bytes, bytes + consumed,
                      copied);
          packet_bytes += copied;
          consumed += copied;
          if (packet_bytes != packet_buffer.size()) {
            continue;
          }
          HlvtdParticipantData decoded{};
          if (decode_hlvtd_participant_data(packet_buffer.data(),
                                            packet_buffer.size(), decoded)) {
            newest_data = decoded;
            ++decoded_in_batch;
          }
          packet_bytes = 0U;
        }
      };

      received_bytes_.fetch_add(static_cast<std::uint64_t>(received));
      consume(receive_buffer.data(), static_cast<std::size_t>(received));

      // Drain everything already queued in the kernel and publish only the
      // newest complete DATA record. This prevents stale state replay after a
      // short Host/network stall while preserving a partial 1109-byte record.
      while (running_) {
        received = ::recv(new_socket, receive_buffer.data(),
                          receive_buffer.size(), MSG_DONTWAIT);
        if (received > 0) {
          received_bytes_.fetch_add(static_cast<std::uint64_t>(received));
          consume(receive_buffer.data(), static_cast<std::size_t>(received));
          continue;
        }
        if (received < 0 && errno == EINTR) {
          continue;
        }
        if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
          break;
        }
        if (received == 0) {
          connection_lost = true;
          break;
        }
        connection_lost = true;
        break;
      }

      if (decoded_in_batch > 0U) {
        decoded_data_packets_.fetch_add(decoded_in_batch);
        if (decoded_in_batch > 1U) {
          skipped_data_packets_.fetch_add(decoded_in_batch - 1U);
        }
        if (data_callback_ && newest_data) {
          data_callback_(*newest_data);
        }
      }
      if (connection_lost) {
        break;
      }
    }

    close_socket();
    if (connection_callback_) {
      connection_callback_(false);
    }
    if (running_) {
      std::this_thread::sleep_for(reconnect_delay_);
    }
  }
}

void HlvtdControlClient::transmit_loop() {
  while (running_) {
    ControlPacket packet{};
    {
      std::unique_lock<std::mutex> lock(pending_mutex_);
      pending_cv_.wait(lock, [this]() {
        return !running_ || (connected_ && pending_command_valid_);
      });
      if (!running_) {
        break;
      }
      packet = pending_command_;
      pending_command_valid_ = false;
    }

    if (send_packet(packet)) {
      ++sent_commands_;
      continue;
    }

    // A failed write invalidates this TCP stream. Closing it also wakes the
    // blocking receiver so that the connection loop can reconnect cleanly.
    close_socket();
  }
}

bool HlvtdControlClient::send_packet(const ControlPacket &packet) {
  std::lock_guard<std::mutex> lock(socket_mutex_);
  if (socket_ < 0 || !connected_) {
    return false;
  }

  std::size_t sent = 0U;
  while (running_ && sent < packet.size()) {
    const auto result =
        ::send(socket_, packet.data() + sent, packet.size() - sent,
               MSG_DONTWAIT | MSG_NOSIGNAL);
    if (result > 0) {
      sent += static_cast<std::size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd descriptor{};
      descriptor.fd = socket_;
      descriptor.events = POLLOUT;
      const int poll_result = ::poll(&descriptor, 1, 50);
      if (poll_result < 0 && errno == EINTR) {
        continue;
      }
      if (poll_result > 0 &&
          (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) == 0) {
        continue;
      }
    }
    return false;
  }
  return sent == packet.size();
}

void HlvtdControlClient::clear_pending_command() {
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_command_valid_ = false;
}

void HlvtdControlClient::close_socket() {
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (socket_ >= 0) {
      ::shutdown(socket_, SHUT_RDWR);
      ::close(socket_);
      socket_ = -1;
    }
    connected_ = false;
  }
  clear_pending_command();
  pending_cv_.notify_all();
}

} // namespace vtd_ros2_bridge
