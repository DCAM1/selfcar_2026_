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

#include "autoware/fixed_route_obstacle_bypass_planner/object_tracker.hpp"

#include "autoware/fixed_route_obstacle_bypass_planner/corridor.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/geometry.hpp"

#include <autoware/object_recognition_utils/predicted_path_utils.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{
namespace bg = boost::geometry;

namespace
{
bool uuid_is_zero(const unique_identifier_msgs::msg::UUID & uuid)
{
  return std::all_of(
    uuid.uuid.begin(), uuid.uuid.end(), [](const auto value) { return value == 0U; });
}

double planar_distance(const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double shape_size_difference(
  const autoware_perception_msgs::msg::Shape & lhs,
  const autoware_perception_msgs::msg::Shape & rhs)
{
  return std::max(
    std::abs(lhs.dimensions.x - rhs.dimensions.x), std::abs(lhs.dimensions.y - rhs.dimensions.y));
}

geometry_msgs::msg::Pose propagate_constant_turn_rate(
  const geometry_msgs::msg::Pose & initial, const geometry_msgs::msg::Twist & local_twist,
  const double time_s)
{
  geometry_msgs::msg::Pose pose = initial;
  const double yaw = tf2::getYaw(initial.orientation);
  const double yaw_rate = local_twist.angular.z;
  const double longitudinal = local_twist.linear.x;
  const double lateral = local_twist.linear.y;

  if (std::abs(yaw_rate) > 1.0e-4 && std::abs(lateral) < 1.0e-3) {
    const double end_yaw = yaw + yaw_rate * time_s;
    const double radius = longitudinal / yaw_rate;
    pose.position.x += radius * (std::sin(end_yaw) - std::sin(yaw));
    pose.position.y -= radius * (std::cos(end_yaw) - std::cos(yaw));
  } else {
    const double velocity_x = std::cos(yaw) * longitudinal - std::sin(yaw) * lateral;
    const double velocity_y = std::sin(yaw) * longitudinal + std::cos(yaw) * lateral;
    pose.position.x += velocity_x * time_s;
    pose.position.y += velocity_y * time_s;
  }
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw + yaw_rate * time_s);
  pose.orientation = tf2::toMsg(orientation);
  return pose;
}
}  // namespace

ObjectTracker::ObjectTracker(Parameters parameters) : parameters_(std::move(parameters))
{
}

void ObjectTracker::set_parameters(Parameters parameters)
{
  parameters_ = std::move(parameters);
}

void ObjectTracker::update(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const rclcpp::Time & received_at)
{
  const rclcpp::Time message_stamp(objects.header.stamp, RCL_ROS_TIME);
  const rclcpp::Time source_stamp =
    message_stamp.nanoseconds() > 0 ? message_stamp : received_at;
  std::unordered_map<std::string, bool> matched;
  for (const auto & [id, unused] : tracks_) {
    (void)unused;
    matched[id] = false;
  }

  for (const auto & object : objects.objects) {
    const std::string source_uuid =
      uuid_is_zero(object.object_id) ? std::string{} : uuid_to_string(object.object_id);
    std::optional<std::string> stable_id;
    if (!source_uuid.empty()) {
      const auto exact = std::find_if(tracks_.begin(), tracks_.end(), [&](const auto & entry) {
        return entry.second.source_uuid == source_uuid && !matched[entry.first];
      });
      if (exact != tracks_.end()) {
        stable_id = exact->first;
      }
    }
    if (!stable_id) {
      stable_id = associate_fallback(object, source_stamp, matched);
    }
    if (!stable_id) {
      stable_id = "generic_unknown_" + std::to_string(next_track_id_++);
      TrackedObstacle new_track;
      new_track.stable_id = *stable_id;
      tracks_.emplace(*stable_id, std::move(new_track));
      matched[*stable_id] = false;
    }

    auto & track = tracks_.at(*stable_id);
    const bool uuid_changed =
      !track.source_uuid.empty() && !source_uuid.empty() && track.source_uuid != source_uuid;
    track.uuid_reassociated = uuid_changed;
    track.source_uuid = source_uuid;
    track.object = object;
    track.source_stamp = source_stamp;
    track.last_seen = received_at;
    track.history.push_back(
      HistorySample{
        track.source_stamp, object.kinematics.initial_pose_with_covariance.pose,
        std::hypot(
          object.kinematics.initial_twist_with_covariance.twist.linear.x,
          object.kinematics.initial_twist_with_covariance.twist.linear.y)});
    while (track.history.size() > 1 &&
           (track.history.back().stamp - track.history.front().stamp).seconds() >
             parameters_.history_window_s) {
      track.history.pop_front();
    }
    matched[*stable_id] = true;
  }
  prune(received_at);
}

void ObjectTracker::prune(const rclcpp::Time & now)
{
  for (auto iterator = tracks_.begin(); iterator != tracks_.end();) {
    if ((now - iterator->second.last_seen).seconds() > parameters_.object_dropout_ttl_s) {
      iterator = tracks_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

void ObjectTracker::clear()
{
  tracks_.clear();
}

const std::unordered_map<std::string, TrackedObstacle> & ObjectTracker::tracks() const
{
  return tracks_;
}

std::optional<std::string> ObjectTracker::associate_fallback(
  const autoware_perception_msgs::msg::PredictedObject & object, const rclcpp::Time & source_stamp,
  const std::unordered_map<std::string, bool> & already_matched) const
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  double best_distance = std::numeric_limits<double>::infinity();
  std::optional<std::string> best_id;
  for (const auto & [id, track] : tracks_) {
    const auto match_iterator = already_matched.find(id);
    if (match_iterator != already_matched.end() && match_iterator->second) {
      continue;
    }
    if (
      shape_size_difference(object.shape, track.object.shape) >
      parameters_.uuid_association_size_tolerance_m) {
      continue;
    }
    const double elapsed = std::max(0.0, (source_stamp - track.source_stamp).seconds());
    const auto predicted_pose = pose_at(track, elapsed, std::nullopt);
    const double distance = planar_distance(object_pose.position, predicted_pose.position);
    if (distance <= parameters_.uuid_association_distance_m && distance < best_distance) {
      best_distance = distance;
      best_id = id;
    }
  }
  return best_id;
}

geometry_msgs::msg::Pose ObjectTracker::pose_at(
  const TrackedObstacle & track, const double relative_from_source_s,
  const std::optional<autoware_perception_msgs::msg::PredictedPath> & predicted_path) const
{
  const double query_time = std::max(0.0, relative_from_source_s);
  if (predicted_path && !predicted_path->path.empty()) {
    const double time_step = rclcpp::Duration(predicted_path->time_step).seconds();
    const double last_time = time_step * static_cast<double>(predicted_path->path.size() - 1);
    if (query_time <= last_time + 1.0e-6) {
      const auto interpolated =
        autoware::object_recognition_utils::calcInterpolatedPose(*predicted_path, query_time);
      if (interpolated) {
        return *interpolated;
      }
    }
    if (time_step > 0.0) {
      return propagate_constant_turn_rate(
        predicted_path->path.back(), track.object.kinematics.initial_twist_with_covariance.twist,
        std::max(0.0, query_time - last_time));
    }
  }
  return propagate_constant_turn_rate(
    track.object.kinematics.initial_pose_with_covariance.pose,
    track.object.kinematics.initial_twist_with_covariance.twist, query_time);
}

bool ObjectTracker::is_uncertain(const TrackedObstacle & track) const
{
  if (
    track.uuid_reassociated ||
    track.history.size() < static_cast<size_t>(parameters_.history_min_samples)) {
    return true;
  }
  double minimum_speed = std::numeric_limits<double>::infinity();
  double maximum_speed = -std::numeric_limits<double>::infinity();
  for (const auto & sample : track.history) {
    minimum_speed = std::min(minimum_speed, sample.speed_mps);
    maximum_speed = std::max(maximum_speed, sample.speed_mps);
    const double yaw_delta = std::atan2(
      std::sin(
        tf2::getYaw(sample.pose.orientation) - tf2::getYaw(track.history.back().pose.orientation)),
      std::cos(
        tf2::getYaw(sample.pose.orientation) - tf2::getYaw(track.history.back().pose.orientation)));
    if (std::abs(yaw_delta) > 0.5) {
      return true;
    }
  }
  return maximum_speed - minimum_speed > 1.0;
}

bool ObjectTracker::is_stationary(const TrackedObstacle & track) const
{
  const double speed = std::hypot(
    track.object.kinematics.initial_twist_with_covariance.twist.linear.x,
    track.object.kinematics.initial_twist_with_covariance.twist.linear.y);
  if (speed > parameters_.stationary_speed_threshold_mps || track.history.empty()) {
    return false;
  }
  return planar_distance(track.history.front().pose.position, track.history.back().pose.position) <=
         parameters_.stationary_displacement_threshold_m;
}

std::vector<OccupancyPolygon> ObjectTracker::occupancies_at(
  const TrackedObstacle & track, const rclcpp::Time & planning_time,
  const double trajectory_time_s, const bool apply_uncertainty_margin) const
{
  std::vector<std::pair<std::string, geometry_msgs::msg::Pose>> hypotheses;
  const double age = std::max(0.0, (planning_time - track.source_stamp).seconds());
  const double query_time = age + std::max(0.0, trajectory_time_s);
  const bool uncertain = is_uncertain(track);
  const bool stationary = is_stationary(track);

  if (stationary || uncertain) {
    hypotheses.emplace_back("stationary", pose_at(track, age, std::nullopt));
  }
  if (!stationary || uncertain) {
    size_t prediction_index = 0;
    for (const auto & predicted_path : track.object.kinematics.predicted_paths) {
      if (predicted_path.confidence < parameters_.min_predicted_path_confidence) {
        ++prediction_index;
        continue;
      }
      hypotheses.emplace_back(
        "predicted_" + std::to_string(prediction_index),
        pose_at(track, query_time, predicted_path));
      ++prediction_index;
    }
    if (
      hypotheses.empty() ||
      std::none_of(hypotheses.begin(), hypotheses.end(), [](const auto & hypothesis) {
        return hypothesis.first.rfind("predicted_", 0) == 0;
      })) {
      hypotheses.emplace_back("constant_velocity", pose_at(track, query_time, std::nullopt));
    }
  }

  const double uncertainty = uncertainty_margin_at(track, planning_time, trajectory_time_s);
  std::vector<OccupancyPolygon> occupancies;
  occupancies.reserve(hypotheses.size());
  for (const auto & [hypothesis_id, pose] : hypotheses) {
    auto polygon = autoware_utils_geometry::to_polygon2d(pose, track.object.shape);
    boost::geometry::correct(polygon);
    if (apply_uncertainty_margin && uncertainty > 0.0) {
      const auto buffered = buffer_polygon(polygon, uncertainty);
      if (!buffered.outer().empty()) {
        polygon = buffered;
      }
    }
    occupancies.push_back(OccupancyPolygon{track.stable_id, hypothesis_id, std::move(polygon)});
  }
  return occupancies;
}

double ObjectTracker::uncertainty_margin_at(
  const TrackedObstacle & track, const rclcpp::Time & planning_time,
  const double trajectory_time_s) const
{
  const double age = std::max(0.0, (planning_time - track.source_stamp).seconds());
  return parameters_.object_collision_margin_m + parameters_.object_uncertainty_base_m +
         parameters_.object_uncertainty_growth_mps *
           (std::max(0.0, trajectory_time_s) + age);
}

MotionState ObjectTracker::classify_motion(
  const TrackedObstacle & track, const autoware::sampler_common::transform::Spline2D & reference,
  const rclcpp::Time & planning_time, double * longitudinal_velocity,
  double * lateral_velocity) const
{
  const double age = std::max(0.0, (planning_time - track.source_stamp).seconds());
  const auto pose = pose_at(track, age, std::nullopt);
  const auto frenet = reference.frenet(Point2d{pose.position.x, pose.position.y});
  const double route_yaw = reference.yaw(frenet.s);
  const double object_yaw = tf2::getYaw(pose.orientation);
  const auto & local_twist = track.object.kinematics.initial_twist_with_covariance.twist;
  const double velocity_x =
    std::cos(object_yaw) * local_twist.linear.x - std::sin(object_yaw) * local_twist.linear.y;
  const double velocity_y =
    std::sin(object_yaw) * local_twist.linear.x + std::cos(object_yaw) * local_twist.linear.y;
  const double route_tangent_x = std::cos(route_yaw);
  const double route_tangent_y = std::sin(route_yaw);
  const double route_normal_x = -route_tangent_y;
  const double route_normal_y = route_tangent_x;
  const double v_s = velocity_x * route_tangent_x + velocity_y * route_tangent_y;
  const double v_d = velocity_x * route_normal_x + velocity_y * route_normal_y;
  if (longitudinal_velocity) {
    *longitudinal_velocity = v_s;
  }
  if (lateral_velocity) {
    *lateral_velocity = v_d;
  }

  if (is_uncertain(track)) {
    return MotionState::UNCERTAIN;
  }
  if (is_stationary(track)) {
    return MotionState::STATIONARY;
  }
  if (std::abs(v_d) >= parameters_.crossing_speed_threshold_mps) {
    return MotionState::CROSSING;
  }
  if (v_s < -parameters_.stationary_speed_threshold_mps) {
    return MotionState::OPPOSITE_DIRECTION;
  }
  return MotionState::SAME_DIRECTION;
}

}  // namespace autoware::fixed_route_obstacle_bypass
