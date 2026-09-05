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

#ifndef AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__CORRIDOR_HPP_
#define AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__CORRIDOR_HPP_

#include "autoware/fixed_route_obstacle_bypass_planner/types.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{

std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & uuid);

std::vector<LaneInterval> select_local_route_lane_intervals(
  const autoware_planning_msgs::msg::LaneletRoute & route,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & ego_pose, double horizon_m, double transition_overlap_m);

std::vector<int64_t> select_local_route_lane_ids(
  const autoware_planning_msgs::msg::LaneletRoute & route,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & ego_pose, double horizon_m, double transition_overlap_m = 0.0);

class RouteCorridorBuilder
{
public:
  void update_map(const autoware_map_msgs::msg::LaneletMapBin & map);
  void update_route(
    const autoware_planning_msgs::msg::LaneletRoute & route, const rclcpp::Time & received_at);
  void update_path(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
    const rclcpp::Time & received_at);

  [[nodiscard]] bool ready() const;
  [[nodiscard]] uint64_t route_generation() const;
  [[nodiscard]] std::string route_uuid() const;
  [[nodiscard]] std::optional<Corridor> build(
    const geometry_msgs::msg::Pose & ego_pose, double horizon_m, double vehicle_half_width_m,
    const Parameters & parameters, std::string * error = nullptr,
    CandidateFailureReason * failure_reason = nullptr) const;
  [[nodiscard]] bool input_is_current(
    const builtin_interfaces::msg::Time & trajectory_stamp, double max_skew_s,
    std::string * error = nullptr, bool require_trajectory_path_stamp_alignment = true) const;

private:
  struct CachedCorridor
  {
    Corridor corridor;
    double path_s{0.0};
    double built_horizon_m{0.0};
    double vehicle_half_width_m{0.0};
    uint64_t route_generation{0};
    std::size_t path_signature{0};
  };

  autoware::route_handler::RouteHandler route_handler_;
  std::optional<autoware_map_msgs::msg::LaneletMapBin> map_;
  std::optional<autoware_planning_msgs::msg::LaneletRoute> route_;
  std::optional<autoware_internal_planning_msgs::msg::PathWithLaneId> path_;
  rclcpp::Time route_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time path_received_at_{0, 0, RCL_ROS_TIME};
  uint64_t route_generation_{0};
  uint64_t path_route_generation_{0};
  std::size_t path_signature_{0};
  mutable std::optional<CachedCorridor> cached_corridor_;
};

}  // namespace autoware::fixed_route_obstacle_bypass

#endif  // AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__CORRIDOR_HPP_
