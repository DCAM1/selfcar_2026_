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

#ifndef AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__PLANNER_NODE_HPP_
#define AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__PLANNER_NODE_HPP_

#include "autoware/fixed_route_obstacle_bypass_planner/corridor.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/object_tracker.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/types.hpp"

#include <autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{

struct CandidateTrajectories
{
  // Desired/regulatory velocity upper bounds for the downstream velocity
  // planner and smoother.
  autoware_planning_msgs::msg::Trajectory downstream_target;
  // Realized timing estimate from the current ego velocity/acceleration, used
  // only for validation.
  autoware_planning_msgs::msg::Trajectory prediction;
};

class FixedRouteObstacleBypassPlannerNode : public rclcpp::Node
{
public:
  explicit FixedRouteObstacleBypassPlannerNode(const rclcpp::NodeOptions & options);

private:
  Parameters parameters_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  RouteCorridorBuilder corridor_builder_;
  ObjectTracker object_tracker_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::shared_ptr<autoware::velocity_smoother::JerkFilteredSmoother> smoother_;

  std::mutex mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr acceleration_;
  autoware_internal_planning_msgs::msg::VelocityLimit::ConstSharedPtr external_velocity_limit_;
  std::optional<autoware_planning_msgs::msg::Trajectory> last_valid_target_;
  std::optional<autoware_planning_msgs::msg::Trajectory> last_valid_prediction_;
  VelocityProfile last_valid_velocity_profile_{VelocityProfile::NORMAL};
  rclcpp::Time last_valid_time_{0, 0, RCL_ROS_TIME};
  uint64_t last_valid_route_generation_{0};
  PassSide committed_side_{PassSide::CENTER};
  std::vector<std::string> committed_objects_;

  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr search_guard_callback_group_;

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::VelocityLimit>::SharedPtr
    velocity_limit_sub_;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::StringStamped>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr processing_time_pub_;
  rclcpp::TimerBase::SharedPtr search_guard_timer_;
  bool objects_received_{false};
  rclcpp::Time last_objects_received_time_{0, 0, RCL_ROS_TIME};
  bool exhaustive_search_in_progress_{false};
  std::optional<autoware_planning_msgs::msg::Trajectory> search_guard_reference_;

  void on_trajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void on_search_guard_timer();
  void publish_state(
    PlanningMode mode, const std::string & reason, size_t candidate_count = 0,
    const CandidateDescriptor * selected = nullptr,
    const std::map<CandidateFailureReason, size_t> * failure_counts = nullptr,
    const CandidateFailureContext * failure_context = nullptr, const Corridor * corridor = nullptr,
    const builtin_interfaces::msg::Time & trajectory_stamp = builtin_interfaces::msg::Time{});
  void publish_markers(
    const Corridor & corridor, const std::vector<ObstacleCluster> & clusters,
    const autoware_planning_msgs::msg::Trajectory * selected);

  [[nodiscard]] std::vector<ObstacleCluster> make_obstacle_clusters(
    const autoware::sampler_common::transform::Spline2D & reference,
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & reference_points,
    const std::vector<double> & reference_s,
    const ObjectTracker & object_tracker, const OccupancyTimeline & occupancy_timeline,
    const rclcpp::Time & planning_time, double initial_s) const;
  [[nodiscard]] std::vector<std::vector<double>> make_lateral_assignments(
    size_t cluster_count) const;
  [[nodiscard]] std::optional<autoware_planning_msgs::msg::Trajectory> make_geometric_candidate(
    const autoware_planning_msgs::msg::Trajectory & reference_trajectory,
    const autoware::sampler_common::transform::Spline2D & reference,
    const std::vector<double> & reference_s, double initial_s, double initial_d,
    const std::vector<ObstacleCluster> & clusters, const std::vector<double> & offsets) const;
  [[nodiscard]] std::optional<CandidateTrajectories> smooth_candidate(
    autoware_planning_msgs::msg::Trajectory candidate, VelocityProfile profile,
    const nav_msgs::msg::Odometry & odometry,
    const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration,
    const std::optional<autoware_internal_planning_msgs::msg::VelocityLimit> &
      external_velocity_limit) const;
};

}  // namespace autoware::fixed_route_obstacle_bypass

#endif  // AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__PLANNER_NODE_HPP_
