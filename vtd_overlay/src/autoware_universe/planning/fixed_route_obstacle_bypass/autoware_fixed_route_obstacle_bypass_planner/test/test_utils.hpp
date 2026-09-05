// Copyright 2026 SelfCar
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include "autoware/fixed_route_obstacle_bypass_planner/geometry.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/types.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <cmath>
#include <cstdint>

namespace autoware::fixed_route_obstacle_bypass::test
{

inline rclcpp::Time ros_time(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1.0e9), RCL_ROS_TIME);
}

inline Polygon2d rectangle(
  const double min_x, const double min_y, const double max_x, const double max_y)
{
  Polygon2d polygon;
  polygon.outer() = {
    Point2d{min_x, min_y}, Point2d{max_x, min_y}, Point2d{max_x, max_y}, Point2d{min_x, max_y},
    Point2d{min_x, min_y}};
  boost::geometry::correct(polygon);
  return polygon;
}

inline geometry_msgs::msg::Pose pose(const double x, const double y, const double yaw = 0.0)
{
  geometry_msgs::msg::Pose output;
  output.position.x = x;
  output.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  output.orientation = tf2::toMsg(quaternion);
  return output;
}

inline autoware::vehicle_info_utils::VehicleInfo vehicle_info()
{
  return autoware::vehicle_info_utils::createVehicleInfo(
    0.35, 0.2, 2.7, 1.6, 0.9, 1.0, 0.1, 0.1, 1.5, 0.6);
}

inline Corridor straight_corridor()
{
  Corridor corridor;
  corridor.original.push_back(rectangle(-5.0, -4.0, 30.0, 4.0));
  corridor.center_space.push_back(rectangle(-3.0, -2.5, 28.0, 2.5));
  corridor.ordered_lane_ids = {10};
  corridor.route_uuid = "test";
  corridor.route_generation = 1;
  return corridor;
}

inline autoware_planning_msgs::msg::Trajectory straight_trajectory(
  const double velocity = 2.0, const double y = 0.0)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  for (size_t i = 0; i <= 20; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose = pose(static_cast<double>(i) * 0.5, y);
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    trajectory.points.push_back(point);
  }
  trajectory.points.back().longitudinal_velocity_mps = 0.0F;
  return trajectory;
}

inline autoware_perception_msgs::msg::PredictedObject object(
  const uint8_t uuid, const double x, const double y, const double yaw,
  const double longitudinal_velocity, const double lateral_velocity = 0.0)
{
  autoware_perception_msgs::msg::PredictedObject output;
  output.object_id.uuid[0] = uuid;
  output.kinematics.initial_pose_with_covariance.pose = pose(x, y, yaw);
  output.kinematics.initial_twist_with_covariance.twist.linear.x = longitudinal_velocity;
  output.kinematics.initial_twist_with_covariance.twist.linear.y = lateral_velocity;
  output.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  output.shape.dimensions.x = 1.0;
  output.shape.dimensions.y = 1.0;
  output.shape.dimensions.z = 1.0;
  return output;
}

}  // namespace autoware::fixed_route_obstacle_bypass::test

#endif  // TEST_UTILS_HPP_
