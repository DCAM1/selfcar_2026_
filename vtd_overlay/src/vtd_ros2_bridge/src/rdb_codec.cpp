#include "vtd_ros2_bridge/rdb_codec.hpp"

#include <algorithm>
#include <cstring>
#include <limits>

namespace vtd_ros2_bridge
{

namespace
{

void set_error(std::string * error, const std::string & value)
{
  if (error) {
    *error = value;
  }
}

}  // namespace

bool parse_rdb_message(
  const std::uint8_t * bytes, const std::size_t size,
  const RdbEntryCallback & callback, std::string * error)
{
  if (!bytes || size < sizeof(RDB_MSG_HDR_t)) {
    set_error(error, "message is smaller than RDB_MSG_HDR_t");
    return false;
  }

  const auto * msg = reinterpret_cast<const RDB_MSG_HDR_t *>(bytes);
  if (msg->magicNo != RDB_MAGIC_NO) {
    set_error(error, "invalid RDB magic number");
    return false;
  }
  if (msg->headerSize < sizeof(RDB_MSG_HDR_t) || msg->headerSize > size) {
    set_error(error, "invalid RDB message header size");
    return false;
  }
  const auto total_size = static_cast<std::uint64_t>(msg->headerSize) + msg->dataSize;
  if (total_size != size) {
    set_error(error, "RDB message size does not match header");
    return false;
  }

  std::size_t offset = msg->headerSize;
  while (offset < size) {
    if (size - offset < sizeof(RDB_MSG_ENTRY_HDR_t)) {
      set_error(error, "truncated RDB entry header");
      return false;
    }
    const auto * entry = reinterpret_cast<const RDB_MSG_ENTRY_HDR_t *>(bytes + offset);
    if (entry->headerSize < sizeof(RDB_MSG_ENTRY_HDR_t) || entry->headerSize > size - offset) {
      set_error(error, "invalid RDB entry header size");
      return false;
    }
    const auto entry_total = static_cast<std::uint64_t>(entry->headerSize) + entry->dataSize;
    if (entry_total > size - offset) {
      set_error(error, "RDB entry extends past message boundary");
      return false;
    }
    if (entry->elementSize != 0U && entry->dataSize % entry->elementSize != 0U) {
      set_error(error, "RDB entry data is not divisible by element size");
      return false;
    }

    callback(
      *msg,
      RdbEntryView{entry, bytes + offset + entry->headerSize, entry->dataSize});
    offset += static_cast<std::size_t>(entry_total);
  }
  return offset == size;
}

std::vector<std::uint8_t> make_driver_control_message(
  const double sim_time, const std::uint32_t frame_no, const RDB_DRIVER_CTRL_t & control)
{
  const std::size_t total_size =
    sizeof(RDB_MSG_HDR_t) + sizeof(RDB_MSG_ENTRY_HDR_t) + sizeof(RDB_DRIVER_CTRL_t);
  std::vector<std::uint8_t> bytes(total_size, 0U);

  auto * msg = reinterpret_cast<RDB_MSG_HDR_t *>(bytes.data());
  msg->magicNo = RDB_MAGIC_NO;
  msg->version = RDB_VERSION;
  msg->headerSize = sizeof(RDB_MSG_HDR_t);
  msg->dataSize = sizeof(RDB_MSG_ENTRY_HDR_t) + sizeof(RDB_DRIVER_CTRL_t);
  msg->frameNo = frame_no;
  msg->simTime = sim_time;

  auto * entry = reinterpret_cast<RDB_MSG_ENTRY_HDR_t *>(bytes.data() + msg->headerSize);
  entry->headerSize = sizeof(RDB_MSG_ENTRY_HDR_t);
  entry->dataSize = sizeof(RDB_DRIVER_CTRL_t);
  entry->elementSize = sizeof(RDB_DRIVER_CTRL_t);
  entry->pkgId = RDB_PKG_ID_DRIVER_CTRL;
  entry->flags = RDB_PKG_FLAG_NONE;

  std::memcpy(bytes.data() + msg->headerSize + entry->headerSize, &control, sizeof(control));
  return bytes;
}

RdbStreamDecoder::RdbStreamDecoder(const std::size_t max_message_size)
: max_message_size_(std::max(max_message_size, sizeof(RDB_MSG_HDR_t)))
{
  buffer_.reserve(64U * 1024U);
}

void RdbStreamDecoder::append(
  const std::uint8_t * bytes, const std::size_t size, const MessageCallback & callback)
{
  if (!bytes || size == 0U) {
    return;
  }
  if (size > max_message_size_ || buffer_.size() > max_message_size_ - size) {
    buffer_.clear();
    ++rejected_messages_;
    return;
  }
  buffer_.insert(buffer_.end(), bytes, bytes + size);

  while (buffer_.size() >= sizeof(RDB_MSG_HDR_t)) {
    if (!discard_until_magic()) {
      return;
    }
    if (buffer_.size() < sizeof(RDB_MSG_HDR_t)) {
      return;
    }

    const auto * msg = reinterpret_cast<const RDB_MSG_HDR_t *>(buffer_.data());
    if (msg->headerSize < sizeof(RDB_MSG_HDR_t)) {
      buffer_.erase(buffer_.begin());
      ++rejected_messages_;
      continue;
    }
    const auto total_size_u64 = static_cast<std::uint64_t>(msg->headerSize) + msg->dataSize;
    if (total_size_u64 > max_message_size_ ||
      total_size_u64 > std::numeric_limits<std::size_t>::max())
    {
      buffer_.erase(buffer_.begin());
      ++rejected_messages_;
      continue;
    }
    const auto total_size = static_cast<std::size_t>(total_size_u64);
    if (buffer_.size() < total_size) {
      return;
    }

    callback(buffer_.data(), total_size);
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total_size));
  }
}

void RdbStreamDecoder::clear()
{
  buffer_.clear();
}

std::size_t RdbStreamDecoder::buffered_size() const noexcept
{
  return buffer_.size();
}

std::uint64_t RdbStreamDecoder::rejected_messages() const noexcept
{
  return rejected_messages_;
}

bool RdbStreamDecoder::discard_until_magic()
{
  const auto magic = static_cast<std::uint16_t>(RDB_MAGIC_NO);
  std::size_t index = 0U;
  for (; index + sizeof(magic) <= buffer_.size(); ++index) {
    std::uint16_t candidate{};
    std::memcpy(&candidate, buffer_.data() + index, sizeof(candidate));
    if (candidate == magic) {
      break;
    }
  }
  if (index > 0U) {
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(index));
    ++rejected_messages_;
  }
  return buffer_.size() >= sizeof(magic);
}

}  // namespace vtd_ros2_bridge
