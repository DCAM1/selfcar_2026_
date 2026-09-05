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

#ifndef AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__OBJECT_TRACKER_HPP_
#define AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__OBJECT_TRACKER_HPP_

#include "autoware/fixed_route_obstacle_bypass_planner/types.hpp"

#include <autoware_sampler_common/transform/spline_transform.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{

struct HistorySample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Pose pose;
  double speed_mps{0.0};
};

struct TrackedObstacle
{
  std::string stable_id;
  std::string source_uuid;
  autoware_perception_msgs::msg::PredictedObject object;
  rclcpp::Time source_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  std::deque<HistorySample> history;
  bool uuid_reassociated{false};
};

class ObjectTracker
{
public:
  explicit ObjectTracker(Parameters parameters);

  void set_parameters(Parameters parameters);
  void update(
    const autoware_perception_msgs::msg::PredictedObjects & objects,
    const rclcpp::Time & received_at);
  void prune(const rclcpp::Time & now);
  void clear();

  [[nodiscard]] const std::unordered_map<std::string, TrackedObstacle> & tracks() const;
  [[nodiscard]] std::vector<OccupancyPolygon> occupancies_at(
    const TrackedObstacle & track, const rclcpp::Time & planning_time,
    double trajectory_time_s, bool apply_uncertainty_margin = true) const;
  [[nodiscard]] double uncertainty_margin_at(
    const TrackedObstacle & track, const rclcpp::Time & planning_time,
    double trajectory_time_s) const;
  [[nodiscard]] MotionState classify_motion(
    const TrackedObstacle & track, const autoware::sampler_common::transform::Spline2D & reference,
    const rclcpp::Time & planning_time, double * longitudinal_velocity = nullptr,
    double * lateral_velocity = nullptr) const;

private:
  Parameters parameters_;
  std::unordered_map<std::string, TrackedObstacle> tracks_;
  uint64_t next_track_id_{1};

  [[nodiscard]] std::optional<std::string> associate_fallback(
    const autoware_perception_msgs::msg::PredictedObject & object,
    const rclcpp::Time & source_stamp,
    const std::unordered_map<std::string, bool> & already_matched) const;
  [[nodiscard]] geometry_msgs::msg::Pose pose_at(
    const TrackedObstacle & track, double relative_from_source_s,
    const std::optional<autoware_perception_msgs::msg::PredictedPath> & predicted_path) const;
  [[nodiscard]] bool is_uncertain(const TrackedObstacle & track) const;
  [[nodiscard]] bool is_stationary(const TrackedObstacle & track) const;
};

}  // namespace autoware::fixed_route_obstacle_bypass

#endif  // AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__OBJECT_TRACKER_HPP_
