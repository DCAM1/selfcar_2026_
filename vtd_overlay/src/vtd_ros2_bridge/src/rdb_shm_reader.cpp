#include "vtd_ros2_bridge/rdb_shm_reader.hpp"

#include <VtdToolkit/viRDBIcd.h>

#include <sys/ipc.h>
#include <sys/shm.h>

#include <algorithm>
#include <cstring>
#include <limits>
#include <thread>
#include <vector>

namespace vtd_ros2_bridge
{

RdbShmReader::RdbShmReader(
  const int key, const std::uint32_t check_mask, MessageCallback message_callback,
  ConnectionCallback connection_callback, const std::chrono::milliseconds poll_interval)
: key_(key),
  check_mask_(check_mask),
  message_callback_(std::move(message_callback)),
  connection_callback_(std::move(connection_callback)),
  poll_interval_(poll_interval)
{
}

RdbShmReader::~RdbShmReader()
{
  stop();
}

void RdbShmReader::start()
{
  if (key_ <= 0 || running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&RdbShmReader::run, this);
}

void RdbShmReader::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (thread_.joinable()) {
    thread_.join();
  }
  detach();
}

bool RdbShmReader::connected() const noexcept
{
  return connected_.load();
}

std::uint64_t RdbShmReader::received_messages() const noexcept
{
  return received_messages_.load();
}

bool RdbShmReader::attach()
{
  shm_id_ = ::shmget(static_cast<key_t>(key_), 0, 0);
  if (shm_id_ < 0) {
    return false;
  }
  shmid_ds status{};
  if (::shmctl(shm_id_, IPC_STAT, &status) != 0) {
    shm_id_ = -1;
    return false;
  }
  void * attached = ::shmat(shm_id_, nullptr, 0);
  if (attached == reinterpret_cast<void *>(-1)) {
    shm_id_ = -1;
    return false;
  }
  shm_ = static_cast<std::uint8_t *>(attached);
  shm_size_ = status.shm_segsz;
  connected_ = true;
  if (connection_callback_) {
    connection_callback_(true);
  }
  return true;
}

void RdbShmReader::detach()
{
  if (shm_) {
    ::shmdt(shm_);
  }
  shm_ = nullptr;
  shm_size_ = 0U;
  shm_id_ = -1;
  const bool was_connected = connected_.exchange(false);
  if (was_connected && connection_callback_) {
    connection_callback_(false);
  }
}

bool RdbShmReader::read_once()
{
  if (!shm_ || shm_size_ < sizeof(RDB_SHM_HDR_t)) {
    return false;
  }
  auto * header = reinterpret_cast<RDB_SHM_HDR_t *>(shm_);
  if (header->headerSize < sizeof(RDB_SHM_HDR_t) || header->headerSize > shm_size_ ||
    header->noBuffers == 0U)
  {
    return false;
  }

  std::vector<RDB_SHM_BUFFER_INFO_t *> buffers;
  std::size_t offset = header->headerSize;
  for (std::uint8_t index = 0; index < header->noBuffers; ++index) {
    if (offset + sizeof(RDB_SHM_BUFFER_INFO_t) > shm_size_) {
      return false;
    }
    auto * info = reinterpret_cast<RDB_SHM_BUFFER_INFO_t *>(shm_ + offset);
    if (info->thisSize < sizeof(RDB_SHM_BUFFER_INFO_t) || offset + info->thisSize > shm_size_ ||
      static_cast<std::uint64_t>(info->offset) + info->bufferSize > shm_size_)
    {
      return false;
    }
    buffers.push_back(info);
    offset += info->thisSize;
  }

  RDB_SHM_BUFFER_INFO_t * selected = nullptr;
  std::uint32_t newest_frame = last_frame_;
  for (auto * info : buffers) {
    const auto flags = info->flags;
    const bool ready = (check_mask_ == 0U || (flags & check_mask_) != 0U) &&
      (flags & RDB_SHM_BUFFER_FLAG_LOCK) == 0U;
    if (!ready || info->bufferSize < sizeof(RDB_MSG_HDR_t)) {
      continue;
    }
    const auto * message = reinterpret_cast<const RDB_MSG_HDR_t *>(shm_ + info->offset);
    if (message->magicNo == RDB_MAGIC_NO && (!selected || message->frameNo > newest_frame)) {
      selected = info;
      newest_frame = message->frameNo;
    }
  }
  if (!selected || newest_frame == last_frame_) {
    return true;
  }

  selected->flags |= RDB_SHM_BUFFER_FLAG_LOCK;
  std::vector<std::vector<std::uint8_t>> messages;
  std::size_t consumed = 0U;
  while (consumed + sizeof(RDB_MSG_HDR_t) <= selected->bufferSize) {
    const auto * source = shm_ + selected->offset + consumed;
    const auto * message = reinterpret_cast<const RDB_MSG_HDR_t *>(source);
    if (message->magicNo != RDB_MAGIC_NO || message->headerSize < sizeof(RDB_MSG_HDR_t)) {
      break;
    }
    const auto total_u64 = static_cast<std::uint64_t>(message->headerSize) + message->dataSize;
    if (total_u64 > selected->bufferSize - consumed ||
      total_u64 > std::numeric_limits<std::size_t>::max())
    {
      break;
    }
    const auto total = static_cast<std::size_t>(total_u64);
    messages.emplace_back(source, source + total);
    consumed += total;
  }
  if (check_mask_ != 0U) {
    selected->flags &= ~check_mask_;
  }
  selected->flags &= ~RDB_SHM_BUFFER_FLAG_LOCK;
  last_frame_ = newest_frame;

  for (const auto & message : messages) {
    ++received_messages_;
    if (message_callback_) {
      message_callback_(message.data(), message.size());
    }
  }
  return true;
}

void RdbShmReader::run()
{
  while (running_) {
    if (!connected_ && !attach()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    if (!read_once()) {
      detach();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      continue;
    }
    std::this_thread::sleep_for(poll_interval_);
  }
}

}  // namespace vtd_ros2_bridge
