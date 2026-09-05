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

#include "autoware/fixed_route_obstacle_bypass_planner/trajectory.hpp"

#include "autoware/fixed_route_obstacle_bypass_planner/geometry.hpp"

#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/algorithms/expand.hpp>
#include <boost/geometry/geometries/box.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{
namespace bg = boost::geometry;

namespace
{
double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double point_distance(const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

autoware_planning_msgs::msg::TrajectoryPoint interpolate_trajectory_point(
  const autoware_planning_msgs::msg::TrajectoryPoint & from,
  const autoware_planning_msgs::msg::TrajectoryPoint & to, const double ratio)
{
  const double clamped = std::clamp(ratio, 0.0, 1.0);
  const auto interpolate = [clamped](const double lhs, const double rhs) {
    return lhs + clamped * (rhs - lhs);
  };
  auto point = from;
  point.pose = interpolate_pose(from.pose, to.pose, clamped);
  point.longitudinal_velocity_mps =
    static_cast<float>(interpolate(from.longitudinal_velocity_mps, to.longitudinal_velocity_mps));
  point.lateral_velocity_mps =
    static_cast<float>(interpolate(from.lateral_velocity_mps, to.lateral_velocity_mps));
  point.acceleration_mps2 =
    static_cast<float>(interpolate(from.acceleration_mps2, to.acceleration_mps2));
  point.heading_rate_rps =
    static_cast<float>(interpolate(from.heading_rate_rps, to.heading_rate_rps));
  point.front_wheel_angle_rad =
    static_cast<float>(interpolate(from.front_wheel_angle_rad, to.front_wheel_angle_rad));
  point.rear_wheel_angle_rad =
    static_cast<float>(interpolate(from.rear_wheel_angle_rad, to.rear_wheel_angle_rad));
  point.time_from_start = builtin_interfaces::msg::Duration();
  return point;
}

double reference_velocity_at_distance(
  const autoware_planning_msgs::msg::Trajectory & reference,
  const std::vector<double> & reference_arc, const size_t reference_start_index,
  const double output_distance)
{
  const double query_s = reference_arc[reference_start_index] + output_distance;
  const auto upper = std::lower_bound(reference_arc.begin(), reference_arc.end(), query_s);
  size_t index = static_cast<size_t>(std::distance(reference_arc.begin(), upper));
  if (index >= reference_arc.size()) {
    index = reference_arc.size() - 1;
  } else if (
    index > reference_start_index &&
    std::abs(reference_arc[index - 1] - query_s) < std::abs(reference_arc[index] - query_s)) {
    --index;
  }
  return std::abs(reference.points[index].longitudinal_velocity_mps);
}

void ensure_minimum_point_count(
  autoware_planning_msgs::msg::Trajectory & trajectory, const size_t minimum_points)
{
  if (trajectory.points.size() >= minimum_points || trajectory.points.size() < 2) {
    return;
  }
  const auto arc = trajectory_arc_lengths(trajectory.points);
  if (arc.back() < 1.0e-6) {
    trajectory.points.resize(minimum_points, trajectory.points.front());
    return;
  }

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resampled;
  resampled.reserve(minimum_points);
  for (size_t i = 0; i < minimum_points; ++i) {
    const double query_s =
      arc.back() * static_cast<double>(i) / static_cast<double>(minimum_points - 1);
    const auto upper = std::lower_bound(arc.begin(), arc.end(), query_s);
    const size_t after = static_cast<size_t>(std::distance(arc.begin(), upper));
    if (after == 0) {
      resampled.push_back(trajectory.points.front());
      continue;
    }
    if (after >= trajectory.points.size()) {
      resampled.push_back(trajectory.points.back());
      continue;
    }
    const size_t before = after - 1;
    const double segment_length = arc[after] - arc[before];
    const double ratio = segment_length > 1.0e-6 ? (query_s - arc[before]) / segment_length : 0.0;
    resampled.push_back(
      interpolate_trajectory_point(trajectory.points[before], trajectory.points[after], ratio));
  }
  trajectory.points = std::move(resampled);
}

size_t nearest_index_unconstrained(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose)
{
  if (points.empty()) {
    return 0;
  }
  const auto cost = [&pose](const auto & point) {
    const double distance = point_distance(point.pose.position, pose.position);
    const double yaw_difference =
      normalize_angle(tf2::getYaw(point.pose.orientation) - tf2::getYaw(pose.orientation));
    constexpr double yaw_cost_scale_m = 5.0;
    return distance * distance +
           yaw_cost_scale_m * yaw_cost_scale_m * yaw_difference * yaw_difference;
  };
  const auto nearest = std::min_element(
    points.begin(), points.end(),
    [&cost](const auto & lhs, const auto & rhs) { return cost(lhs) < cost(rhs); });
  return static_cast<size_t>(std::distance(points.begin(), nearest));
}

double triangle_curvature(
  const geometry_msgs::msg::Point & previous, const geometry_msgs::msg::Point & current,
  const geometry_msgs::msg::Point & next)
{
  const double a = point_distance(previous, current);
  const double b = point_distance(current, next);
  const double c = point_distance(previous, next);
  const double denominator = a * b * c;
  if (denominator < 1.0e-6) {
    return 0.0;
  }
  const double cross = (current.x - previous.x) * (next.y - previous.y) -
                       (current.y - previous.y) * (next.x - previous.x);
  return 2.0 * cross / denominator;
}

std::map<std::string, std::vector<Polygon2d>> group_occupancies(
  const std::vector<OccupancyPolygon> & occupancies)
{
  std::map<std::string, std::vector<Polygon2d>> grouped;
  for (const auto & occupancy : occupancies) {
    grouped[occupancy.hypothesis_id].push_back(occupancy.polygon);
  }
  return grouped;
}
}  // namespace

std::vector<double> trajectory_arc_lengths(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points)
{
  std::vector<double> arc_lengths(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    arc_lengths[i] =
      arc_lengths[i - 1] + point_distance(points[i - 1].pose.position, points[i].pose.position);
  }
  return arc_lengths;
}

std::optional<size_t> nearest_trajectory_index(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_distance_m,
  const double max_yaw_difference_rad)
{
  if (points.empty()) {
    return std::nullopt;
  }
  const double ego_yaw = tf2::getYaw(pose.orientation);
  double best_distance = std::numeric_limits<double>::infinity();
  std::optional<size_t> best_index;
  for (size_t i = 0; i < points.size(); ++i) {
    const double distance = point_distance(points[i].pose.position, pose.position);
    const double yaw_difference =
      std::abs(normalize_angle(tf2::getYaw(points[i].pose.orientation) - ego_yaw));
    if (
      distance <= max_distance_m && yaw_difference <= max_yaw_difference_rad &&
      distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

void assign_time_from_start(
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const double initial_velocity_mps, const double max_acceleration_mps2)
{
  if (points.empty()) {
    return;
  }
  points.front().time_from_start = rclcpp::Duration::from_seconds(0.0);
  double elapsed = 0.0;
  double previous_velocity = std::max(0.0, std::abs(initial_velocity_mps));
  for (size_t i = 1; i < points.size(); ++i) {
    const double distance = point_distance(points[i - 1].pose.position, points[i].pose.position);
    const double current_velocity =
      std::max(0.0, static_cast<double>(std::abs(points[i].longitudinal_velocity_mps)));
    const double velocity_sum = previous_velocity + current_velocity;
    double delta_time = 0.0;
    if (distance > 1.0e-6 && velocity_sum > 1.0e-3) {
      delta_time = 2.0 * distance / velocity_sum;
    } else if (distance > 1.0e-6 && max_acceleration_mps2 > 1.0e-3) {
      delta_time = std::sqrt(2.0 * distance / max_acceleration_mps2);
    }
    elapsed += std::clamp(delta_time, 0.0, 10.0);
    points[i].time_from_start = rclcpp::Duration::from_seconds(elapsed);
    previous_velocity = current_velocity;
  }
}

autoware_planning_msgs::msg::Trajectory make_timed_validation_trajectory(
  const autoware_planning_msgs::msg::Trajectory & command_trajectory,
  const double measured_velocity_mps, const double max_acceleration_mps2)
{
  auto timed = command_trajectory;
  assign_time_from_start(timed.points, measured_velocity_mps, max_acceleration_mps2);
  return timed;
}

std::optional<autoware_planning_msgs::msg::Trajectory> align_trajectory_to_ego(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, const double horizon_m, const double max_distance_m,
  const double max_yaw_difference_rad)
{
  if (trajectory.points.size() < 2 || horizon_m <= 0.0) {
    return std::nullopt;
  }
  const auto nearest =
    nearest_trajectory_index(trajectory.points, ego_pose, max_distance_m, max_yaw_difference_rad);
  if (!nearest) {
    return std::nullopt;
  }

  struct Projection
  {
    size_t segment_index{0};
    double ratio{0.0};
    double distance{std::numeric_limits<double>::infinity()};
  };
  std::optional<Projection> best;
  const size_t first_segment = *nearest > 0 ? *nearest - 1 : 0;
  const size_t last_segment = std::min(*nearest, trajectory.points.size() - 2);
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  for (size_t segment_index = first_segment; segment_index <= last_segment; ++segment_index) {
    const auto & from = trajectory.points[segment_index].pose.position;
    const auto & to = trajectory.points[segment_index + 1].pose.position;
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length < 1.0e-8) {
      continue;
    }
    const double ratio = std::clamp(
      ((ego_pose.position.x - from.x) * dx + (ego_pose.position.y - from.y) * dy) / squared_length,
      0.0, 1.0);
    const auto projected_pose = interpolate_pose(
      trajectory.points[segment_index].pose, trajectory.points[segment_index + 1].pose, ratio);
    const double distance = point_distance(projected_pose.position, ego_pose.position);
    const double yaw_difference =
      std::abs(normalize_angle(tf2::getYaw(projected_pose.orientation) - ego_yaw));
    if (
      distance <= max_distance_m && yaw_difference <= max_yaw_difference_rad &&
      (!best || distance < best->distance)) {
      best = Projection{segment_index, ratio, distance};
    }
  }
  if (!best) {
    return std::nullopt;
  }

  autoware_planning_msgs::msg::Trajectory aligned;
  aligned.header = trajectory.header;
  aligned.points.reserve(trajectory.points.size() - best->segment_index);
  aligned.points.push_back(interpolate_trajectory_point(
    trajectory.points[best->segment_index], trajectory.points[best->segment_index + 1],
    best->ratio));

  const size_t next_index =
    best->ratio >= 1.0 - 1.0e-6 ? best->segment_index + 2 : best->segment_index + 1;
  double accumulated_distance = 0.0;
  for (size_t i = next_index; i < trajectory.points.size(); ++i) {
    const auto & previous = aligned.points.back();
    const double segment_distance =
      point_distance(previous.pose.position, trajectory.points[i].pose.position);
    if (segment_distance < 1.0e-6) {
      continue;
    }
    if (accumulated_distance + segment_distance > horizon_m) {
      const double ratio = (horizon_m - accumulated_distance) / segment_distance;
      if (ratio > 1.0e-6) {
        aligned.points.push_back(
          interpolate_trajectory_point(previous, trajectory.points[i], ratio));
      }
      break;
    }
    aligned.points.push_back(trajectory.points[i]);
    aligned.points.back().time_from_start = builtin_interfaces::msg::Duration();
    accumulated_distance += segment_distance;
    if (accumulated_distance >= horizon_m - 1.0e-6) {
      break;
    }
  }
  if (aligned.points.size() < 2) {
    return std::nullopt;
  }
  return aligned;
}

StartupValidationResult check_unexpected_zero_velocity(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const autoware_planning_msgs::msg::Trajectory & regulatory_reference,
  const double ego_velocity_mps, const Parameters & parameters)
{
  StartupValidationResult result;
  if (trajectory.points.size() < 2 || regulatory_reference.points.empty()) {
    result.failure_reason = CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY;
    return result;
  }
  const auto output_arc = trajectory_arc_lengths(trajectory.points);
  const auto reference_arc = trajectory_arc_lengths(regulatory_reference.points);
  const size_t reference_start_index =
    nearest_index_unconstrained(regulatory_reference.points, trajectory.points.front().pose);

  size_t check_begin = 0;
  if (std::abs(ego_velocity_mps) <= parameters.startup_enter_velocity_mps) {
    const size_t confirmation_points =
      static_cast<size_t>(std::max<int64_t>(1, parameters.startup_moving_confirmation_points));
    std::optional<size_t> release_index;
    for (size_t i = 0; i + confirmation_points <= trajectory.points.size(); ++i) {
      const bool confirmed = std::all_of(
        std::next(trajectory.points.begin(), static_cast<std::ptrdiff_t>(i)),
        std::next(trajectory.points.begin(), static_cast<std::ptrdiff_t>(i + confirmation_points)),
        [&](const auto & point) {
          return std::abs(point.longitudinal_velocity_mps) >=
                 parameters.startup_release_velocity_mps;
        });
      if (confirmed) {
        release_index = i;
        break;
      }
    }

    if (!release_index) {
      bool reference_requires_motion = false;
      for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
        if (output_arc.back() - output_arc[i] <= parameters.endpoint_stop_tolerance_m) {
          continue;
        }
        if (
          reference_velocity_at_distance(
            regulatory_reference, reference_arc, reference_start_index, output_arc[i]) >
          parameters.zero_velocity_epsilon_mps) {
          reference_requires_motion = true;
          result.failure_index = i;
          result.failure_s = output_arc[i];
          result.velocity_mps = std::abs(trajectory.points[i].longitudinal_velocity_mps);
          result.reference_velocity_mps = reference_velocity_at_distance(
            regulatory_reference, reference_arc, reference_start_index, output_arc[i]);
          break;
        }
      }
      if (reference_requires_motion) {
        result.failure_reason = CandidateFailureReason::ALL_ZERO_TRAJECTORY;
        return result;
      }
      result.valid = true;
      return result;
    }

    result.startup_release_index = release_index;
    const double release_time = duration_seconds(trajectory.points[*release_index].time_from_start);
    result.startup_release_time_s = release_time;
    if (
      output_arc[*release_index] > parameters.max_startup_prefix_distance_m ||
      release_time > parameters.max_startup_prefix_time_s) {
      result.failure_reason = CandidateFailureReason::STARTUP_PREFIX_TOO_LONG;
      result.failure_index = *release_index;
      result.failure_s = output_arc[*release_index];
      result.velocity_mps = std::abs(trajectory.points[*release_index].longitudinal_velocity_mps);
      result.reference_velocity_mps = reference_velocity_at_distance(
        regulatory_reference, reference_arc, reference_start_index, output_arc[*release_index]);
      return result;
    }
    for (size_t i = 1; i <= *release_index; ++i) {
      const double previous_velocity = std::abs(trajectory.points[i - 1].longitudinal_velocity_mps);
      const double velocity = std::abs(trajectory.points[i].longitudinal_velocity_mps);
      if (velocity + parameters.startup_velocity_drop_tolerance_mps < previous_velocity) {
        result.failure_reason = CandidateFailureReason::STARTUP_PROFILE_INVALID;
        result.failure_index = i;
        result.failure_s = output_arc[i];
        result.velocity_mps = velocity;
        result.reference_velocity_mps = reference_velocity_at_distance(
          regulatory_reference, reference_arc, reference_start_index, output_arc[i]);
        return result;
      }
    }
    check_begin = *release_index;
  }

  for (size_t i = check_begin; i + 1 < trajectory.points.size(); ++i) {
    const double velocity = std::abs(trajectory.points[i].longitudinal_velocity_mps);
    if (velocity > parameters.zero_velocity_epsilon_mps) {
      continue;
    }
    if (output_arc.back() - output_arc[i] <= parameters.endpoint_stop_tolerance_m) {
      continue;
    }
    const double reference_velocity = reference_velocity_at_distance(
      regulatory_reference, reference_arc, reference_start_index, output_arc[i]);
    if (reference_velocity > parameters.zero_velocity_epsilon_mps) {
      result.failure_reason = CandidateFailureReason::UNEXPECTED_INTERIOR_ZERO;
      result.failure_index = i;
      result.failure_s = output_arc[i];
      result.velocity_mps = velocity;
      result.reference_velocity_mps = reference_velocity;
      return result;
    }
  }
  result.valid = true;
  return result;
}

bool has_nonterminal_regulatory_stop(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & forward_start_pose, const Parameters & parameters)
{
  if (trajectory.points.empty()) {
    return false;
  }
  const auto arc = trajectory_arc_lengths(trajectory.points);
  const size_t start_index = nearest_index_unconstrained(trajectory.points, forward_start_pose);
  for (size_t i = start_index; i < trajectory.points.size(); ++i) {
    if (
      std::abs(trajectory.points[i].longitudinal_velocity_mps) <=
        parameters.zero_velocity_epsilon_mps &&
      arc.back() - arc[i] > parameters.endpoint_stop_tolerance_m) {
      return true;
    }
  }
  return false;
}

double progress_at_time(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double time_s)
{
  if (trajectory.points.empty()) {
    return 0.0;
  }
  const auto arc_lengths = trajectory_arc_lengths(trajectory.points);
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const double previous_time = duration_seconds(trajectory.points[i - 1].time_from_start);
    const double current_time = duration_seconds(trajectory.points[i].time_from_start);
    if (current_time >= time_s) {
      const double ratio =
        current_time > previous_time
          ? std::clamp((time_s - previous_time) / (current_time - previous_time), 0.0, 1.0)
          : 0.0;
      return arc_lengths[i - 1] + ratio * (arc_lengths[i] - arc_lengths[i - 1]);
    }
  }
  return arc_lengths.back();
}

OccupancyTimeline make_occupancy_timeline(
  const ObjectTracker & object_tracker, const rclcpp::Time & planning_time,
  const Parameters & parameters)
{
  OccupancyTimeline timeline;
  timeline.interval_s = std::max(0.05, parameters.prediction_sampling_interval_s);
  const size_t bin_count = static_cast<size_t>(
    std::ceil(parameters.collision_horizon_s / timeline.interval_s));
  timeline.bins.reserve(bin_count);
  for (size_t bin_index = 0; bin_index < bin_count; ++bin_index) {
    OccupancyTimeBin bin;
    bin.begin_time_s = static_cast<double>(bin_index) * timeline.interval_s;
    bin.end_time_s =
      std::min(parameters.collision_horizon_s, bin.begin_time_s + timeline.interval_s);
    const double middle_time = 0.5 * (bin.begin_time_s + bin.end_time_s);
    for (const auto & [track_id, track] : object_tracker.tracks()) {
      auto hypothesis_polygons = group_occupancies(
        object_tracker.occupancies_at(track, planning_time, bin.begin_time_s, false));
      for (const auto time : {middle_time, bin.end_time_s}) {
        for (const auto & [hypothesis, polygons] : group_occupancies(
               object_tracker.occupancies_at(track, planning_time, time, false))) {
          auto & destination = hypothesis_polygons[hypothesis];
          destination.insert(destination.end(), polygons.begin(), polygons.end());
        }
      }
      const double uncertainty_margin =
        object_tracker.uncertainty_margin_at(track, planning_time, bin.end_time_s);
      for (auto & [hypothesis, polygons] : hypothesis_polygons) {
        auto sweep = convex_sweep(polygons);
        if (sweep.outer().empty()) {
          continue;
        }
        Box2d envelope;
        bg::envelope(sweep, envelope);
        bin.occupancies.push_back(SweptOccupancy{
          track_id, hypothesis, std::move(sweep), envelope, uncertainty_margin});
      }
    }
    timeline.bins.push_back(std::move(bin));
  }
  return timeline;
}

ValidationResult validate_trajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const Corridor & corridor,
  const ObjectTracker & object_tracker, const rclcpp::Time & planning_time,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const Parameters & parameters,
  const bool check_kinematics, const bool require_shrunken_center_space,
  const bool check_longitudinal_kinematics, const bool check_corridor,
  const bool check_collision, const OccupancyTimeline * occupancy_timeline)
{
  ValidationResult result;
  result.corridor_valid = true;
  result.collision_free = true;
  result.kinematically_valid = true;
  if (input_trajectory.points.size() < 2) {
    result.reason = "forward trajectory is empty";
    result.corridor_valid = false;
    result.failure_reason = CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY;
    return result;
  }
  if (check_corridor && corridor.original.empty()) {
    result.reason = "route corridor is empty or disconnected";
    result.corridor_valid = false;
    result.failure_reason = CandidateFailureReason::CORRIDOR_DISCONNECTED;
    return result;
  }

  auto trajectory = input_trajectory;
  const bool missing_time =
    std::all_of(trajectory.points.begin(), trajectory.points.end(), [](const auto & point) {
      return point.time_from_start.sec == 0 && point.time_from_start.nanosec == 0U;
    });
  if (missing_time) {
    assign_time_from_start(
      trajectory.points, trajectory.points.front().longitudinal_velocity_mps,
      parameters.max_longitudinal_acceleration_mps2);
  }

  const auto point_containment_started = std::chrono::steady_clock::now();
  if (check_corridor) {
    for (size_t i = 0; i < trajectory.points.size(); ++i) {
      ++result.point_containment_count;
      const Point2d center{
        trajectory.points[i].pose.position.x, trajectory.points[i].pose.position.y};
      if (require_shrunken_center_space && !bg::covered_by(center, corridor.center_space)) {
        result.corridor_valid = false;
        result.reason = "trajectory center leaves the shrunken route corridor";
        result.trajectory_index = i;
        result.failure_reason = CandidateFailureReason::TRAJECTORY_OUTSIDE_CORRIDOR;
        return result;
      }
      const auto footprint = vehicle_footprint(trajectory.points[i].pose, vehicle_info);
      if (!polygon_covered_by(
            footprint, corridor.original, parameters.corridor_area_tolerance_m2)) {
        result.corridor_valid = false;
        result.reason = "rotated vehicle footprint leaves the route corridor";
        result.trajectory_index = i;
        result.failure_reason = CandidateFailureReason::TRAJECTORY_OUTSIDE_CORRIDOR;
        return result;
      }
    }
  }
  const auto point_containment_completed = std::chrono::steady_clock::now();
  result.point_containment_time_ms =
    std::chrono::duration<double, std::milli>(
      point_containment_completed - point_containment_started)
      .count();

  std::optional<OccupancyTimeline> local_occupancy_timeline;
  if (check_collision && occupancy_timeline == nullptr) {
    local_occupancy_timeline =
      make_occupancy_timeline(object_tracker, planning_time, parameters);
    occupancy_timeline = &*local_occupancy_timeline;
  }

  const auto swept_validation_started = std::chrono::steady_clock::now();
  const auto check_collision_interval = [&result, occupancy_timeline](
                                          const Polygon2d & ego_sweep, const double begin_time,
                                          const double end_time,
                                          const size_t trajectory_index) {
    if (
      occupancy_timeline == nullptr || occupancy_timeline->bins.empty() ||
      occupancy_timeline->interval_s <= 0.0) {
      return true;
    }
    boost::geometry::model::box<Point2d> ego_envelope;
    bg::envelope(ego_sweep, ego_envelope);
    const auto clamped_bin_index = [&](const double time) {
      const auto raw_index = static_cast<size_t>(
        std::floor(std::max(0.0, time) / occupancy_timeline->interval_s));
      return std::min(raw_index, occupancy_timeline->bins.size() - 1);
    };
    const size_t first_bin = clamped_bin_index(begin_time);
    const size_t last_bin = clamped_bin_index(std::max(begin_time, end_time - 1.0e-9));
    for (size_t bin_index = first_bin; bin_index <= last_bin; ++bin_index) {
      for (const auto & occupancy : occupancy_timeline->bins[bin_index].occupancies) {
        const double envelope_clearance = bg::distance(ego_envelope, occupancy.envelope);
        if (envelope_clearance > occupancy.uncertainty_margin_m) {
          // Axis-aligned envelope distance is a lower bound on polygon
          // distance, so this broad-phase rejection cannot hide a collision.
          result.minimum_clearance_m = std::min(
            result.minimum_clearance_m,
            envelope_clearance - occupancy.uncertainty_margin_m);
          continue;
        }
        const double raw_clearance = bg::distance(ego_sweep, occupancy.polygon);
        result.minimum_clearance_m =
          std::min(
            result.minimum_clearance_m,
            std::max(0.0, raw_clearance - occupancy.uncertainty_margin_m));
        if (raw_clearance <= occupancy.uncertainty_margin_m) {
          result.collision_free = false;
          result.reason = "swept collision with generic UNKNOWN occupancy";
          result.object_id = occupancy.track_id;
          result.trajectory_index = trajectory_index;
          result.trajectory_time_s = 0.5 * (begin_time + end_time);
          result.ego_envelope_min_x = ego_envelope.min_corner().x();
          result.ego_envelope_min_y = ego_envelope.min_corner().y();
          result.ego_envelope_max_x = ego_envelope.max_corner().x();
          result.ego_envelope_max_y = ego_envelope.max_corner().y();
          result.object_envelope_min_x = occupancy.envelope.min_corner().x();
          result.object_envelope_min_y = occupancy.envelope.min_corner().y();
          result.object_envelope_max_x = occupancy.envelope.max_corner().x();
          result.object_envelope_max_y = occupancy.envelope.max_corner().y();
          result.failure_reason = CandidateFailureReason::COLLISION_FAILURE;
          return false;
        }
      }
    }
    return true;
  };

  std::optional<std::pair<geometry_msgs::msg::Pose, double>> stationary_stop;
  for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    const auto & from = trajectory.points[i];
    const auto & to = trajectory.points[i + 1];
    const double from_time = duration_seconds(from.time_from_start);
    const double to_time = duration_seconds(to.time_from_start);
    if (from_time > parameters.collision_horizon_s) {
      break;
    }
    const double interval_end_time = std::min(to_time, parameters.collision_horizon_s);
    geometry_msgs::msg::Pose interval_end_pose = to.pose;
    if (to_time > parameters.collision_horizon_s && to_time > from_time + 1.0e-6) {
      const double horizon_ratio =
        (parameters.collision_horizon_s - from_time) / (to_time - from_time);
      interval_end_pose = interpolate_pose(from.pose, to.pose, horizon_ratio);
    }
    const bool segment_moves =
      std::abs(from.longitudinal_velocity_mps) > parameters.zero_velocity_epsilon_mps ||
      std::abs(to.longitudinal_velocity_mps) > parameters.zero_velocity_epsilon_mps;
    const bool moves_later = std::any_of(
      std::next(trajectory.points.begin(), static_cast<std::ptrdiff_t>(i + 2)),
      trajectory.points.end(), [&](const auto & point) {
        return std::abs(point.longitudinal_velocity_mps) > parameters.zero_velocity_epsilon_mps;
      });
    if (!segment_moves && !moves_later) {
      stationary_stop = std::make_pair(from.pose, from_time);
      break;
    }

    const auto time_steps = static_cast<size_t>(std::ceil(
      std::max(0.0, interval_end_time - from_time) /
      std::max(0.05, parameters.prediction_sampling_interval_s)));
    const auto ego_sweeps = swept_footprints(
      from.pose, interval_end_pose, vehicle_info, parameters.sweep_max_step_m,
      parameters.sweep_max_yaw_step_rad, 0.0, time_steps);
    for (size_t sweep_index = 0; sweep_index < ego_sweeps.size(); ++sweep_index) {
      ++result.swept_footprint_count;
      const double begin_ratio = static_cast<double>(sweep_index) / ego_sweeps.size();
      const double end_ratio = static_cast<double>(sweep_index + 1) / ego_sweeps.size();
      const double begin_time =
        from_time + begin_ratio * (interval_end_time - from_time);
      const double end_time = from_time + end_ratio * (interval_end_time - from_time);

      if (
        check_corridor && !polygon_covered_by(
            ego_sweeps[sweep_index], corridor.original, parameters.corridor_area_tolerance_m2)) {
        result.corridor_valid = false;
        result.reason = "swept vehicle footprint leaves the route corridor";
        result.trajectory_index = i;
        result.failure_reason = CandidateFailureReason::TRAJECTORY_OUTSIDE_CORRIDOR;
        return result;
      }

      if (
        check_collision && !check_collision_interval(
            ego_sweeps[sweep_index], begin_time, end_time, i)) {
        return result;
      }
    }
    if (to_time >= parameters.collision_horizon_s) {
      break;
    }
  }

  if (
    !stationary_stop && std::abs(trajectory.points.back().longitudinal_velocity_mps) <=
                          parameters.zero_velocity_epsilon_mps) {
    stationary_stop = std::make_pair(
      trajectory.points.back().pose, duration_seconds(trajectory.points.back().time_from_start));
  }

  if (
    check_collision && stationary_stop &&
    stationary_stop->second < parameters.collision_horizon_s) {
    const auto stopped_footprint = vehicle_footprint(stationary_stop->first, vehicle_info);
    const double interval = std::max(0.05, parameters.prediction_sampling_interval_s);
    for (double begin_time = stationary_stop->second; begin_time < parameters.collision_horizon_s;
         begin_time += interval) {
      const double end_time = std::min(parameters.collision_horizon_s, begin_time + interval);
      if (!check_collision_interval(
            stopped_footprint, begin_time, end_time, trajectory.points.size() - 1)) {
        return result;
      }
    }
  }

  const auto swept_validation_completed = std::chrono::steady_clock::now();
  result.swept_validation_time_ms =
    std::chrono::duration<double, std::milli>(
      swept_validation_completed - swept_validation_started)
      .count();

  if (check_kinematics && trajectory.points.size() >= 3) {
    double previous_acceleration = trajectory.points.front().acceleration_mps2;
    double previous_steering_angle = 0.0;
    bool has_previous_steering_angle = false;
    for (size_t i = 1; i + 1 < trajectory.points.size(); ++i) {
      const double curvature = triangle_curvature(
        trajectory.points[i - 1].pose.position, trajectory.points[i].pose.position,
        trajectory.points[i + 1].pose.position);
      const double velocity = std::abs(trajectory.points[i].longitudinal_velocity_mps);
      const double lateral_acceleration = velocity * velocity * std::abs(curvature);
      const double steering_angle = std::abs(std::atan(vehicle_info.wheel_base_m * curvature));
      if (
        lateral_acceleration > parameters.max_lateral_acceleration_mps2 + 0.05 ||
        steering_angle > vehicle_info.max_steer_angle_rad + 0.01) {
        result.kinematically_valid = false;
        result.reason = "lateral acceleration or steering limit exceeded";
        result.trajectory_index = i;
        result.failure_reason = CandidateFailureReason::LATERAL_LIMIT_FAILURE;
        return result;
      }

      const double previous_time = duration_seconds(trajectory.points[i - 1].time_from_start);
      const double current_time = duration_seconds(trajectory.points[i].time_from_start);
      const double delta_time = current_time - previous_time;
      if (delta_time > 1.0e-3) {
        const double signed_steering_angle = std::atan(vehicle_info.wheel_base_m * curvature);
        if (
          has_previous_steering_angle &&
          std::abs(signed_steering_angle - previous_steering_angle) / delta_time >
            parameters.max_steering_rate_radps + 0.02) {
          result.kinematically_valid = false;
          result.reason = "steering rate limit exceeded";
          result.trajectory_index = i;
          result.failure_reason = CandidateFailureReason::LATERAL_LIMIT_FAILURE;
          return result;
        }
        previous_steering_angle = signed_steering_angle;
        has_previous_steering_angle = true;
        const double previous_velocity =
          std::abs(trajectory.points[i - 1].longitudinal_velocity_mps);
        const double acceleration = (velocity - previous_velocity) / delta_time;
        const double jerk = (acceleration - previous_acceleration) / delta_time;
        if (
          check_longitudinal_kinematics &&
          (acceleration > parameters.max_longitudinal_acceleration_mps2 + 0.1 ||
           acceleration < parameters.min_longitudinal_acceleration_mps2 - 0.1 ||
           jerk > parameters.max_longitudinal_jerk_mps3 + 0.2 ||
           jerk < parameters.min_longitudinal_jerk_mps3 - 0.2)) {
          result.kinematically_valid = false;
          result.reason = "longitudinal acceleration or jerk limit exceeded";
          result.trajectory_index = i;
          result.failure_reason = CandidateFailureReason::LONGITUDINAL_LIMIT_FAILURE;
          return result;
        }
        previous_acceleration = acceleration;
      }
    }
  }

  result.valid = true;
  result.reason = "valid";
  result.failure_reason = CandidateFailureReason::NONE;
  return result;
}

ValidationResult validate_emergency_trajectory(
  const autoware_planning_msgs::msg::Trajectory & input_trajectory, const Corridor & corridor,
  const ObjectTracker & object_tracker, const rclcpp::Time & planning_time,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const Parameters & parameters)
{
  auto reachable = input_trajectory;
  if (reachable.points.empty()) {
    ValidationResult result;
    result.reason = "emergency trajectory is empty";
    result.failure_reason = CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY;
    return result;
  }

  size_t stop_index = reachable.points.size() - 1;
  for (size_t i = 0; i < reachable.points.size(); ++i) {
    if (
      std::abs(reachable.points[i].longitudinal_velocity_mps) <=
      parameters.zero_velocity_epsilon_mps) {
      stop_index = i;
      break;
    }
  }
  reachable.points.resize(stop_index + 1);
  if (reachable.points.size() == 1) {
    reachable.points.push_back(reachable.points.front());
  }
  assign_time_from_start(
    reachable.points, reachable.points.front().longitudinal_velocity_mps,
    std::abs(parameters.emergency_deceleration_mps2));
  return validate_trajectory(
    reachable, corridor, object_tracker, planning_time, vehicle_info, parameters, false, false);
}

std::optional<autoware_planning_msgs::msg::Trajectory> crop_downstream_target(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & stamp,
  const Parameters & parameters)
{
  auto cropped = align_trajectory_to_ego(
    trajectory, odometry.pose.pose, parameters.planning_horizon_m,
    parameters.nearest_distance_threshold_m, parameters.nearest_yaw_threshold_rad);
  if (!cropped) {
    return std::nullopt;
  }
  if (trajectory_arc_lengths(cropped->points).back() < parameters.last_valid_min_horizon_m) {
    return std::nullopt;
  }
  cropped->header.stamp = stamp;
  for (auto & point : cropped->points) {
    point.time_from_start = builtin_interfaces::msg::Duration{};
  }
  return cropped;
}

std::optional<autoware_planning_msgs::msg::Trajectory> crop_and_retime_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & stamp,
  const Parameters & parameters)
{
  auto cropped = align_trajectory_to_ego(
    trajectory, odometry.pose.pose, parameters.planning_horizon_m,
    parameters.nearest_distance_threshold_m, parameters.nearest_yaw_threshold_rad);
  if (!cropped) {
    return std::nullopt;
  }
  cropped->header.stamp = stamp;
  cropped->points.front().longitudinal_velocity_mps =
    static_cast<float>(odometry.twist.twist.linear.x);
  if (trajectory_arc_lengths(cropped->points).back() < parameters.last_valid_min_horizon_m) {
    return std::nullopt;
  }
  assign_time_from_start(
    cropped->points, odometry.twist.twist.linear.x, parameters.max_longitudinal_acceleration_mps2);
  return cropped;
}

autoware_planning_msgs::msg::Trajectory make_emergency_stop_trajectory(
  const autoware_planning_msgs::msg::Trajectory & reference,
  const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & stamp,
  const Parameters & parameters)
{
  const double initial_velocity = std::abs(odometry.twist.twist.linear.x);
  const double deceleration = std::min(-1.0e-3, parameters.emergency_deceleration_mps2);
  const bool stationary = initial_velocity <= parameters.zero_velocity_epsilon_mps;
  const double braking_distance =
    initial_velocity * initial_velocity / (2.0 * std::abs(deceleration));
  const double requested_length = stationary
                                    ? parameters.emergency_stationary_length_m
                                    : braking_distance + parameters.emergency_braking_margin_m;
  auto cropped = align_trajectory_to_ego(
    reference, odometry.pose.pose,
    std::min(parameters.planning_horizon_m, std::max(0.5, requested_length)),
    parameters.nearest_distance_threshold_m, parameters.nearest_yaw_threshold_rad);
  autoware_planning_msgs::msg::Trajectory emergency;
  if (cropped) {
    emergency = *cropped;
    emergency.header.stamp = stamp;
  } else {
    emergency.header = reference.header;
    emergency.header.stamp = stamp;
    const double yaw = tf2::getYaw(odometry.pose.pose.orientation);
    const size_t point_count =
      static_cast<size_t>(std::max<int64_t>(2, parameters.emergency_min_points));
    const double point_interval = requested_length / static_cast<double>(point_count - 1);
    for (size_t i = 0; i < point_count; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint point;
      point.pose = odometry.pose.pose;
      point.pose.position.x += static_cast<double>(i) * point_interval * std::cos(yaw);
      point.pose.position.y += static_cast<double>(i) * point_interval * std::sin(yaw);
      emergency.points.push_back(point);
    }
  }

  ensure_minimum_point_count(
    emergency, static_cast<size_t>(std::max<int64_t>(2, parameters.emergency_min_points)));

  const auto arc_lengths = trajectory_arc_lengths(emergency.points);
  for (size_t i = 0; i < emergency.points.size(); ++i) {
    const double squared_velocity =
      initial_velocity * initial_velocity + 2.0 * deceleration * arc_lengths[i];
    emergency.points[i].longitudinal_velocity_mps =
      static_cast<float>(std::sqrt(std::max(0.0, squared_velocity)));
    emergency.points[i].acceleration_mps2 =
      emergency.points[i].longitudinal_velocity_mps > parameters.zero_velocity_epsilon_mps
        ? static_cast<float>(deceleration)
        : 0.0F;
  }
  if (!emergency.points.empty()) {
    emergency.points.back().longitudinal_velocity_mps = 0.0F;
  }
  if (!stationary) {
    const auto first_stop = std::find_if(
      std::next(emergency.points.begin()), emergency.points.end(), [&](const auto & point) {
        return std::abs(point.longitudinal_velocity_mps) <= parameters.zero_velocity_epsilon_mps;
      });
    if (first_stop != emergency.points.end()) {
      emergency.points.erase(std::next(first_stop), emergency.points.end());
    }
  }
  assign_time_from_start(
    emergency.points, initial_velocity, std::abs(parameters.emergency_deceleration_mps2));
  return emergency;
}

}  // namespace autoware::fixed_route_obstacle_bypass
