#pragma once

#include <VtdToolkit/viRDBIcd.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace vtd_ros2_bridge
{

struct RdbEntryView
{
  const RDB_MSG_ENTRY_HDR_t * header{};
  const std::uint8_t * data{};
  std::size_t data_size{};
};

using RdbEntryCallback =
  std::function<void(const RDB_MSG_HDR_t &, const RdbEntryView &)>;

bool parse_rdb_message(
  const std::uint8_t * bytes, std::size_t size,
  const RdbEntryCallback & callback, std::string * error = nullptr);

std::vector<std::uint8_t> make_driver_control_message(
  double sim_time, std::uint32_t frame_no, const RDB_DRIVER_CTRL_t & control);

class RdbStreamDecoder
{
public:
  using MessageCallback = std::function<void(const std::uint8_t *, std::size_t)>;

  explicit RdbStreamDecoder(std::size_t max_message_size = 512U * 1024U * 1024U);

  void append(const std::uint8_t * bytes, std::size_t size, const MessageCallback & callback);
  void clear();
  std::size_t buffered_size() const noexcept;
  std::uint64_t rejected_messages() const noexcept;

private:
  bool discard_until_magic();

  std::vector<std::uint8_t> buffer_;
  std::size_t max_message_size_;
  std::uint64_t rejected_messages_{0};
};

}  // namespace vtd_ros2_bridge
