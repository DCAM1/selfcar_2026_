#include "vtd_ros2_bridge/hlvtd_lidar_codec.hpp"
#include "vtd_ros2_bridge/rdb_udp_receiver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace vtd_ros2_bridge {

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

class VtdLidarNode : public rclcpp::Node {
public:
  VtdLidarNode() : Node("vtd_lidar") {
    bind_address_ =
        declare_parameter<std::string>("lidar_udp.bind_address", "0.0.0.0");
    port_ = declare_parameter<int>("lidar_udp.port", 9912);
    socket_receive_buffer_bytes_ = declare_parameter<int>(
        "lidar_udp.socket_receive_buffer_bytes", 4 * 1024 * 1024);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar_link");
    map_offset_x_ = declare_parameter<double>("map_offset.x", 0.0);
    map_offset_y_ = declare_parameter<double>("map_offset.y", 0.0);
    map_offset_z_ = declare_parameter<double>("map_offset.z", 0.0);
    map_yaw_offset_ = declare_parameter<double>("map_offset.yaw", 0.0);
    const auto pointcloud_topic = declare_parameter<std::string>(
        "topics.pointcloud", "/sensing/lidar/concatenated/pointcloud");

    pointcloud_pub_ = create_publisher<PointCloud2>(
        pointcloud_topic, rclcpp::SensorDataQoS().reliable());
    receiver_ = std::make_unique<RdbUdpReceiver>(
        bind_address_, port_,
        [this](const std::uint8_t *data, const std::size_t size) {
          on_datagram(data, size);
        },
        [this](const bool bound) {
          RCLCPP_INFO(get_logger(), "VTD LiDAR UDP channel %s (%s:%d)",
                      bound ? "bound" : "unbound", bind_address_.c_str(),
                      port_);
        },
        socket_receive_buffer_bytes_);
    receiver_->start();

    RCLCPP_INFO(get_logger(),
                "VTD LiDAR bridge ready: udp=%s:%d topic=%s",
                bind_address_.c_str(), port_, pointcloud_topic.c_str());
  }

  ~VtdLidarNode() override {
    if (receiver_) {
      receiver_->stop();
    }
  }

private:
  struct PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
  };

  void on_datagram(const std::uint8_t *data, const std::size_t size) {
    if (!is_hlvtd_lidar_packet(data, size)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Rejected non-IVHL packet on LiDAR UDP channel");
      return;
    }

    HlvtdLidarPacketView packet;
    std::string error;
    if (!parse_hlvtd_lidar_packet(data, size, packet, &error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Rejected IVHL LiDAR packet: %s", error.c_str());
      return;
    }

    auto frame = assembler_.add_packet(packet, &error);
    if (!error.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Rejected IVHL LiDAR frame: %s", error.c_str());
      return;
    }
    if (frame) {
      publish_frame(*frame);
    }
  }

  std::array<double, 3> map_position(const float x, const float y,
                                     const float z) const {
    const double cosine = std::cos(map_yaw_offset_);
    const double sine = std::sin(map_yaw_offset_);
    return {cosine * x - sine * y + map_offset_x_,
            sine * x + cosine * y + map_offset_y_, z + map_offset_z_};
  }

  void publish_frame(const HlvtdLidarFrame &frame) {
    std::vector<PointXYZI> points;
    points.reserve(frame.points.size());
    for (const auto &point : frame.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) ||
          (point.x == 0.0F && point.y == 0.0F && point.z == 0.0F)) {
        continue;
      }
      if (frame.coordinate_mode == kHlvtdLidarCoordinatesWorld) {
        const auto mapped = map_position(point.x, point.y, point.z);
        points.push_back({static_cast<float>(mapped[0]),
                          static_cast<float>(mapped[1]),
                          static_cast<float>(mapped[2]), 0.0F});
      } else {
        points.push_back({point.x, point.y, point.z, 0.0F});
      }
    }
    if (points.empty()) {
      return;
    }

    PointCloud2 cloud;
    const auto timestamp = frame.timestamp_nanoseconds;
    cloud.header.stamp.sec =
        static_cast<std::int32_t>(timestamp / 1000000000ULL);
    cloud.header.stamp.nanosec =
        static_cast<std::uint32_t>(timestamp % 1000000000ULL);
    cloud.header.frame_id = frame.coordinate_mode == kHlvtdLidarCoordinatesWorld
                                ? map_frame_
                                : lidar_frame_;
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = sizeof(PointXYZI);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(4U);
    const std::array<std::string, 4> names{"x", "y", "z", "intensity"};
    for (std::size_t index = 0; index < cloud.fields.size(); ++index) {
      cloud.fields[index].name = names[index];
      cloud.fields[index].offset =
          static_cast<std::uint32_t>(index * sizeof(float));
      cloud.fields[index].datatype = PointField::FLOAT32;
      cloud.fields[index].count = 1U;
    }
    cloud.data.resize(cloud.row_step);
    std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
    pointcloud_pub_->publish(cloud);
  }

  std::string bind_address_;
  int port_{};
  int socket_receive_buffer_bytes_{};
  std::string map_frame_;
  std::string lidar_frame_;
  double map_offset_x_{};
  double map_offset_y_{};
  double map_offset_z_{};
  double map_yaw_offset_{};
  std::unique_ptr<RdbUdpReceiver> receiver_;
  HlvtdLidarFrameAssembler assembler_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;
};

} // namespace vtd_ros2_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vtd_ros2_bridge::VtdLidarNode>());
  rclcpp::shutdown();
  return 0;
}
