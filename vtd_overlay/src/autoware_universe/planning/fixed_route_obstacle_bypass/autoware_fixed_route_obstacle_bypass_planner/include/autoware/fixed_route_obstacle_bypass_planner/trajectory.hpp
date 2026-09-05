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

#ifndef AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__TRAJECTORY_HPP_
#define AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__TRAJECTORY_HPP_

#include "autoware/fixed_route_obstacle_bypass_planner/object_tracker.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/types.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{

std::vector<double> trajectory_arc_lengths(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points);

std::optional<size_t> nearest_trajectory_index(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, double max_distance_m, double max_yaw_difference_rad);

void assign_time_from_start(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, double initial_velocity_mps,
  double max_acceleration_mps2);

// Build a time-parameterized copy for swept-occupancy validation without
// changing the post-smoother command velocities. The first command velocity
// may intentionally be Autoware's engage velocity while the measured ego
// velocity is still zero.
autoware_planning_msgs::msg::Trajectory make_timed_validation_trajectory(
  const autoware_planning_msgs::msg::Trajectory & command_trajectory,
  double measured_velocity_mps, double max_acceleration_mps2);

std::optional<autoware_planning_msgs::msg::Trajectory> align_trajectory_to_ego(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, double horizon_m, double max_distance_m,
  double max_yaw_difference_rad);

StartupValidationResult check_unexpected_zero_velocity(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware_planning_msgs::msg::Trajectory & regulatory_reference, double ego_velocity_mps,
  const Parameters & parameters);

bool has_nonterminal_regulatory_stop(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & forward_start_pose, const Parameters & parameters);

double progress_at_time(const autoware_planning_msgs::msg::Trajectory & trajectory, double time_s);

OccupancyTimeline make_occupancy_timeline(
  const ObjectTracker & object_tracker, const rclcpp::Time & planning_time,
  const Parameters & parameters);

ValidationResult validate_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const Corridor & corridor,
  const ObjectTracker & object_tracker, const rclcpp::Time & planning_time,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const Parameters & parameters,
  bool check_kinematics = true, bool require_shrunken_center_space = true,
  bool check_longitudinal_kinematics = true, bool check_corridor = true,
  bool check_collision = true, const OccupancyTimeline * occupancy_timeline = nullptr);

ValidationResult validate_emergency_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const Corridor & corridor,
  const ObjectTracker & object_tracker, const rclcpp::Time & planning_time,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const Parameters & parameters);

// Crop a desired velocity target for reuse upstream of the global velocity
// smoother. Unlike a realized trajectory, this must never receive the measured
// ego velocity: a startup zero in this message would be interpreted downstream
// as a stop point.
std::optional<autoware_planning_msgs::msg::Trajectory> crop_downstream_target(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & stamp,
  const Parameters & parameters);

std::optional<autoware_planning_msgs::msg::Trajectory> crop_and_retime_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & stamp,
  const Parameters & parameters);

autoware_planning_msgs::msg::Trajectory make_emergency_stop_trajectory(
  const autoware_planning_msgs::msg::Trajectory & reference,
  const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & stamp,
  const Parameters & parameters);

}  // namespace autoware::fixed_route_obstacle_bypass

#endif  // AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__TRAJECTORY_HPP_
