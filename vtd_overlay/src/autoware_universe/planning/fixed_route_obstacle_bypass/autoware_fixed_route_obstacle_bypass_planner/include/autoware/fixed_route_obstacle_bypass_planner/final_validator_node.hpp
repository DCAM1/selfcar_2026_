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

#ifndef AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__FINAL_VALIDATOR_NODE_HPP_
#define AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__FINAL_VALIDATOR_NODE_HPP_

#include "autoware/fixed_route_obstacle_bypass_planner/corridor.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/object_tracker.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/types.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace autoware::fixed_route_obstacle_bypass
{

class FixedRouteFinalTrajectoryValidatorNode : public rclcpp::Node
{
public:
  explicit FixedRouteFinalTrajectoryValidatorNode(const rclcpp::NodeOptions & options);

private:
  Parameters parameters_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  RouteCorridorBuilder corridor_builder_;
  ObjectTracker object_tracker_;

  std::mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr validation_callback_group_;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr regulatory_reference_;
  std::map<int64_t, autoware_planning_msgs::msg::Trajectory>
    regulatory_references_by_stamp_;
  std::map<int64_t, std::string> planner_states_by_stamp_;
  // Preserve the post-smoother command contract. A measured startup zero must
  // never be written into this trajectory because downstream control uses the
  // smoother's engage velocity to leave standstill.
  std::optional<autoware_planning_msgs::msg::Trajectory> last_valid_target_;
  rclcpp::Time last_valid_time_{0, 0, RCL_ROS_TIME};
  uint64_t last_valid_route_generation_{0};
  bool objects_received_{false};
  rclcpp::Time last_objects_received_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr reference_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::StringStamped>::SharedPtr
    planner_state_sub_;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  void on_trajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void publish_diagnostic(
    PlanningMode mode, const ValidationResult & result, const std::string & action,
    const Corridor * corridor = nullptr);
};

}  // namespace autoware::fixed_route_obstacle_bypass

#endif  // AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__FINAL_VALIDATOR_NODE_HPP_
