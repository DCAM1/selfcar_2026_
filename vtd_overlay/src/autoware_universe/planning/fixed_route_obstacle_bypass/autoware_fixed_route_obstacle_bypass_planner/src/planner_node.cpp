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

#include "autoware/fixed_route_obstacle_bypass_planner/planner_node.hpp"

#include "autoware/fixed_route_obstacle_bypass_planner/parameters.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/trajectory.hpp"

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_frenet_planner/frenet_planner.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/covered_by.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace autoware::fixed_route_obstacle_bypass
{
namespace
{
using autoware::sampler_common::transform::Spline2D;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

double point_distance(const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

struct PolylineProjection
{
  double s{0.0};
  double d{0.0};
  double squared_distance{std::numeric_limits<double>::infinity()};
};

PolylineProjection project_to_polyline(
  const std::vector<TrajectoryPoint> & points, const std::vector<double> & arc_lengths,
  const Point2d & query)
{
  PolylineProjection best;
  if (points.size() < 2 || arc_lengths.size() != points.size()) {
    return best;
  }
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto & from = points[i].pose.position;
    const auto & to = points[i + 1].pose.position;
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length < 1.0e-8) {
      continue;
    }
    const double ratio = std::clamp(
      ((query.x() - from.x) * dx + (query.y() - from.y) * dy) / squared_length, 0.0, 1.0);
    const double projected_x = from.x + ratio * dx;
    const double projected_y = from.y + ratio * dy;
    const double offset_x = query.x() - projected_x;
    const double offset_y = query.y() - projected_y;
    const double squared_distance = offset_x * offset_x + offset_y * offset_y;
    if (squared_distance >= best.squared_distance) {
      continue;
    }
    const double length = std::sqrt(squared_length);
    best.s = arc_lengths[i] + ratio * length;
    best.d = (-dy * offset_x + dx * offset_y) / length;
    best.squared_distance = squared_distance;
  }
  return best;
}

double interpolate_value(
  const std::vector<double> & base_s, const std::vector<TrajectoryPoint> & points,
  const double query_s, const bool velocity)
{
  if (base_s.empty() || points.empty()) {
    return 0.0;
  }
  const auto upper = std::upper_bound(base_s.begin(), base_s.end(), query_s);
  if (upper == base_s.begin()) {
    return velocity ? points.front().longitudinal_velocity_mps : points.front().pose.position.z;
  }
  if (upper == base_s.end()) {
    return velocity ? points.back().longitudinal_velocity_mps : points.back().pose.position.z;
  }
  const size_t after = static_cast<size_t>(std::distance(base_s.begin(), upper));
  const size_t before = after - 1;
  const double span = base_s[after] - base_s[before];
  const double ratio = span > 1.0e-6 ? (query_s - base_s[before]) / span : 0.0;
  const double before_value =
    velocity ? points[before].longitudinal_velocity_mps : points[before].pose.position.z;
  const double after_value =
    velocity ? points[after].longitudinal_velocity_mps : points[after].pose.position.z;
  return before_value + std::clamp(ratio, 0.0, 1.0) * (after_value - before_value);
}

PassSide side_from_offsets(const std::vector<double> & offsets)
{
  if (offsets.empty()) {
    return PassSide::CENTER;
  }
  const auto largest = std::max_element(
    offsets.begin(), offsets.end(),
    [](const double lhs, const double rhs) { return std::abs(lhs) < std::abs(rhs); });
  if (std::abs(*largest) < 1.0e-3) {
    return PassSide::CENTER;
  }
  return *largest > 0.0 ? PassSide::LEFT : PassSide::RIGHT;
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

std::string lane_ids_to_string(const std::vector<int64_t> & lane_ids)
{
  std::ostringstream stream;
  for (size_t i = 0; i < lane_ids.size(); ++i) {
    if (i > 0) {
      stream << ',';
    }
    stream << lane_ids[i];
  }
  return stream.str();
}
}  // namespace

FixedRouteObstacleBypassPlannerNode::FixedRouteObstacleBypassPlannerNode(
  const rclcpp::NodeOptions & options)
: Node("fixed_route_obstacle_bypass_planner", options), object_tracker_(parameters_)
{
  parameters_ = autoware::fixed_route_obstacle_bypass::declare_parameters(*this);
  object_tracker_.set_parameters(parameters_);
  if (!parameters_.exclusive_mode_verified) {
    throw std::runtime_error(
      "fixed-route bypass requires launch-time "
      "exclusion of legacy obstacle trajectory writers");
  }
  if (
    parameters_.lateral_offsets_m.empty() || parameters_.max_geometry_candidates < 1 ||
    parameters_.path_sampling_interval_m <= 0.0 ||
    parameters_.prediction_sampling_interval_s <= 0.0 || parameters_.creep_velocity_mps <= 0.0 ||
    parameters_.max_longitudinal_acceleration_mps2 <= 0.0 ||
    parameters_.startup_release_velocity_mps <= parameters_.zero_velocity_epsilon_mps ||
    parameters_.startup_enter_velocity_mps < parameters_.zero_velocity_epsilon_mps ||
    parameters_.startup_moving_confirmation_points < 1 ||
    parameters_.max_startup_prefix_distance_m <= 0.0 ||
    parameters_.max_startup_prefix_time_s <= 0.0 || parameters_.emergency_min_points < 2 ||
    parameters_.smoother_engage_velocity_mps <= 0.0 ||
    parameters_.smoother_engage_acceleration_mps2 <= 0.0 ||
    parameters_.smoother_engage_exit_ratio < 0.0 ||
    parameters_.smoother_engage_exit_ratio > 1.0 ||
    parameters_.smoother_stop_distance_to_prohibit_engage_m < 0.0 ||
    parameters_.emergency_stationary_length_m <= 0.0 ||
    parameters_.corridor_path_clip_half_width_m <= 0.0 ||
    parameters_.corridor_transition_overlap_m < 0.0) {
    throw std::invalid_argument("invalid fixed-route bypass sampling or velocity parameters");
  }

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", rclcpp::QoS{1});
  state_pub_ = create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
    "~/output/state", rclcpp::QoS{1}.transient_local());
  diagnostics_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/output/diagnostics", rclcpp::QoS{1});
  markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", rclcpp::QoS{1});
  processing_time_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time", rclcpp::QoS{1});
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(processing_time_pub_);
  smoother_ =
    std::make_shared<autoware::velocity_smoother::JerkFilteredSmoother>(*this, time_keeper_);
  smoother_->setWheelBase(vehicle_info_.wheel_base_m);

  input_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  planning_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  search_guard_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions input_subscription_options;
  input_subscription_options.callback_group = input_callback_group_;
  rclcpp::SubscriptionOptions planning_subscription_options;
  planning_subscription_options.callback_group = planning_callback_group_;

  const auto durable_qos = rclcpp::QoS{1}.transient_local().reliable();
  map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", durable_qos,
    [this](const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      corridor_builder_.update_map(*msg);
    },
    input_subscription_options);
  route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/route", durable_qos,
    [this](const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto previous_generation = corridor_builder_.route_generation();
      corridor_builder_.update_route(*msg, now());
      if (corridor_builder_.route_generation() != previous_generation) {
        object_tracker_.clear();
        objects_received_ = false;
        last_valid_target_.reset();
        last_valid_prediction_.reset();
        committed_objects_.clear();
        committed_side_ = PassSide::CENTER;
      }
    },
    input_subscription_options);
  path_sub_ = create_subscription<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "~/input/path_with_lane_id", rclcpp::QoS{1},
    [this](const autoware_internal_planning_msgs::msg::PathWithLaneId::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      corridor_builder_.update_path(*msg, now());
    },
    input_subscription_options);
  objects_sub_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    [this](const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      objects_received_ = true;
      last_objects_received_time_ = now();
      object_tracker_.update(*msg, last_objects_received_time_);
    },
    input_subscription_options);
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      odometry_ = msg;
    }, input_subscription_options);
  acceleration_sub_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/acceleration", rclcpp::QoS{1},
    [this](const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      acceleration_ = msg;
    },
    input_subscription_options);
  velocity_limit_sub_ = create_subscription<autoware_internal_planning_msgs::msg::VelocityLimit>(
    "~/input/external_velocity_limit", rclcpp::QoS{1},
    [this](const autoware_internal_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      external_velocity_limit_ = msg;
    },
    input_subscription_options);
  trajectory_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&FixedRouteObstacleBypassPlannerNode::on_trajectory, this, std::placeholders::_1),
    planning_subscription_options);
  search_guard_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&FixedRouteObstacleBypassPlannerNode::on_search_guard_timer, this),
    search_guard_callback_group_);

  RCLCPP_INFO(
    get_logger(),
    "fixed-route class-free bypass enabled; legacy "
    "obstacle writers are launch-excluded");
}

std::vector<ObstacleCluster> FixedRouteObstacleBypassPlannerNode::make_obstacle_clusters(
  const Spline2D & reference,
  const std::vector<TrajectoryPoint> & reference_points, const std::vector<double> & reference_s,
  const ObjectTracker & object_tracker, const OccupancyTimeline & occupancy_timeline,
  const rclcpp::Time & planning_time, const double initial_s) const
{
  // Predicted occupancy is sampled many times over the collision horizon.
  // Projecting every polygon corner onto every 0.1 m reference segment made
  // this broad-phase clustering dominate the planning cycle.  A 0.5 m
  // polyline is sufficient for the broad phase; expand the resulting bounds
  // by its maximum chord length so it cannot exclude an occupancy that the
  // exact swept-collision validator would otherwise see.
  std::vector<TrajectoryPoint> projection_points;
  std::vector<double> projection_s;
  projection_points.reserve(reference_points.size());
  projection_s.reserve(reference_s.size());
  constexpr double projection_interval_m = 0.5;
  for (size_t i = 0; i < reference_points.size(); ++i) {
    const bool is_endpoint = i == 0 || i + 1 == reference_points.size();
    if (
      is_endpoint || projection_s.empty() ||
      reference_s[i] - projection_s.back() >= projection_interval_m) {
      projection_points.push_back(reference_points[i]);
      projection_s.push_back(reference_s[i]);
    }
  }
  double projection_margin_m = 0.0;
  for (size_t i = 1; i < projection_s.size(); ++i) {
    projection_margin_m =
      std::max(projection_margin_m, projection_s[i] - projection_s[i - 1]);
  }

  std::unordered_map<std::string, ObstacleCluster> clusters_by_track;
  for (const auto & bin : occupancy_timeline.bins) {
    for (const auto & occupancy : bin.occupancies) {
      auto & cluster = clusters_by_track[occupancy.track_id];
      if (cluster.object_ids.empty()) {
        cluster.min_s = std::numeric_limits<double>::infinity();
        cluster.max_s = -std::numeric_limits<double>::infinity();
        cluster.object_ids.push_back(occupancy.track_id);
      }
        double occupancy_min_s = std::numeric_limits<double>::infinity();
        double occupancy_max_s = -std::numeric_limits<double>::infinity();
        double minimum_d = std::numeric_limits<double>::infinity();
        double maximum_d = -std::numeric_limits<double>::infinity();
        for (const auto & point : occupancy.polygon.outer()) {
          const auto projection = project_to_polyline(projection_points, projection_s, point);
          if (!std::isfinite(projection.squared_distance)) {
            continue;
          }
          occupancy_min_s = std::min(occupancy_min_s, projection.s);
          occupancy_max_s = std::max(occupancy_max_s, projection.s);
          minimum_d = std::min(minimum_d, projection.d);
          maximum_d = std::max(maximum_d, projection.d);
        }
        if (!std::isfinite(occupancy_min_s)) {
          continue;
        }
        const double lateral_distance = minimum_d <= 0.0 && maximum_d >= 0.0
                                          ? 0.0
                                          : std::min(std::abs(minimum_d), std::abs(maximum_d));
        // The route corridor is itself clipped by this path-relative mask.
        // Anything intersecting the true corridor must pass this conservative
        // broad-phase test; exact footprint/corridor and timed collision
        // checks remain in validate_trajectory().
        if (
          lateral_distance >
          parameters_.corridor_path_clip_half_width_m + occupancy.uncertainty_margin_m +
            projection_margin_m) {
          continue;
        }
        cluster.min_s = std::min(
          cluster.min_s,
          occupancy_min_s - occupancy.uncertainty_margin_m - projection_margin_m);
        cluster.max_s = std::max(
          cluster.max_s,
          occupancy_max_s + occupancy.uncertainty_margin_m + projection_margin_m);
    }
  }

  std::vector<ObstacleCluster> raw_clusters;
  raw_clusters.reserve(clusters_by_track.size());
  for (auto & [track_id, cluster] : clusters_by_track) {
    if (!std::isfinite(cluster.min_s)) {
      continue;
    }
    if (
      !std::isfinite(cluster.min_s) ||
      cluster.max_s < initial_s - parameters_.commitment_release_distance_m ||
      cluster.min_s >
        initial_s + parameters_.planning_horizon_m + parameters_.obstacle_longitudinal_margin_m) {
      continue;
    }
    raw_clusters.push_back(cluster);

    if (parameters_.debug_logging) {
      double longitudinal_velocity = 0.0;
      double lateral_velocity = 0.0;
      const auto track = object_tracker.tracks().find(track_id);
      if (track == object_tracker.tracks().end()) {
        continue;
      }
      const auto state = object_tracker.classify_motion(
        track->second, reference, planning_time, &longitudinal_velocity, &lateral_velocity);
      RCLCPP_INFO(
        get_logger(),
        "generic occupancy id=%s semantic=UNKNOWN motion=%s v_s=%.2f "
        "v_d=%.2f conflict_s=[%.2f, %.2f]",
        track_id.c_str(), to_string(state), longitudinal_velocity, lateral_velocity, cluster.min_s,
        cluster.max_s);
    }
  }

  std::sort(raw_clusters.begin(), raw_clusters.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.min_s < rhs.min_s;
  });
  std::vector<ObstacleCluster> clusters;
  for (const auto & cluster : raw_clusters) {
    if (
      !clusters.empty() &&
      cluster.min_s <= clusters.back().max_s + parameters_.cluster_longitudinal_gap_m) {
      clusters.back().max_s = std::max(clusters.back().max_s, cluster.max_s);
      clusters.back().object_ids.insert(
        clusters.back().object_ids.end(), cluster.object_ids.begin(), cluster.object_ids.end());
    } else {
      clusters.push_back(cluster);
    }
  }
  return clusters;
}

std::vector<std::vector<double>> FixedRouteObstacleBypassPlannerNode::make_lateral_assignments(
  const size_t cluster_count) const
{
  if (cluster_count == 0U) {
    return {{}};
  }
  auto offsets = parameters_.lateral_offsets_m;
  std::sort(offsets.begin(), offsets.end(), [](const double lhs, const double rhs) {
    if (std::abs(lhs) == std::abs(rhs)) {
      return lhs > rhs;
    }
    return std::abs(lhs) < std::abs(rhs);
  });
  std::vector<std::vector<double>> assignments{{}};
  const size_t limit =
    static_cast<size_t>(std::max<int64_t>(1, parameters_.max_geometry_candidates));
  for (size_t cluster_index = 0; cluster_index < cluster_count; ++cluster_index) {
    std::vector<std::vector<double>> expanded;
    for (const auto & assignment : assignments) {
      for (const double offset : offsets) {
        auto candidate = assignment;
        candidate.push_back(offset);
        expanded.push_back(std::move(candidate));
      }
    }
    std::stable_sort(expanded.begin(), expanded.end(), [](const auto & lhs, const auto & rhs) {
      const auto cost = [](const auto & values) {
        return std::accumulate(values.begin(), values.end(), 0.0, [](double sum, double value) {
          return sum + std::abs(value);
        });
      };
      return cost(lhs) < cost(rhs);
    });
    if (expanded.size() > limit) {
      expanded.resize(limit);
    }
    assignments = std::move(expanded);
  }
  // Always retain uniform full-clearance candidates. A beam ranked only by
  // absolute shift can otherwise discard the only feasible side for multiple
  // clustered obstacles.
  std::vector<std::vector<double>> prioritized;
  prioritized.reserve(limit);
  for (const double offset : offsets) {
    if (prioritized.size() >= limit) {
      break;
    }
    std::vector<double> uniform(cluster_count, offset);
    prioritized.push_back(std::move(uniform));
  }
  for (auto & assignment : assignments) {
    if (
      prioritized.size() < limit &&
      std::find(prioritized.begin(), prioritized.end(), assignment) == prioritized.end()) {
      prioritized.push_back(std::move(assignment));
    }
  }
  return prioritized;
}

std::optional<Trajectory> FixedRouteObstacleBypassPlannerNode::make_geometric_candidate(
  const Trajectory & reference_trajectory, const Spline2D & reference,
  const std::vector<double> & reference_s, const double initial_s, const double initial_d,
  const std::vector<ObstacleCluster> & clusters, const std::vector<double> & offsets) const
{
  if (
    reference_trajectory.points.size() < 2 ||
    reference_s.size() != reference_trajectory.points.size()) {
    return std::nullopt;
  }
  const bool has_shift = std::any_of(
    offsets.begin(), offsets.end(), [](const double offset) { return std::abs(offset) > 1.0e-3; });
  const double candidate_end_s =
    std::min(reference.lastS(), initial_s + parameters_.planning_horizon_m);
  if (!has_shift) {
    Trajectory output = reference_trajectory;
    const auto begin = std::lower_bound(reference_s.begin(), reference_s.end(), initial_s - 1.0);
    const size_t begin_index = static_cast<size_t>(std::distance(reference_s.begin(), begin));
    const auto end = std::upper_bound(reference_s.begin(), reference_s.end(), candidate_end_s);
    const size_t end_index = static_cast<size_t>(std::distance(reference_s.begin(), end));
    output.points.erase(
      std::next(output.points.begin(), static_cast<std::ptrdiff_t>(end_index)),
      output.points.end());
    output.points.erase(
      output.points.begin(),
      std::next(output.points.begin(), static_cast<std::ptrdiff_t>(begin_index)));
    return output.points.size() >= 2 ? std::optional<Trajectory>{output} : std::nullopt;
  }

  std::vector<std::pair<double, double>> waypoints;
  double current_s = std::clamp(initial_s, reference.firstS(), candidate_end_s);
  waypoints.emplace_back(current_s, initial_d);
  double current_d = initial_d;
  for (size_t i = 0; i < clusters.size() && i < offsets.size(); ++i) {
    const double shift_start_s = std::clamp(
      clusters[i].min_s - parameters_.shift_prepare_distance_m, current_s, candidate_end_s);
    if (shift_start_s > current_s + parameters_.path_sampling_interval_m) {
      waypoints.emplace_back(shift_start_s, current_d);
      current_s = shift_start_s;
    }
    const double target_s = std::clamp(
      clusters[i].min_s - parameters_.obstacle_longitudinal_margin_m,
      current_s + parameters_.path_sampling_interval_m, candidate_end_s);
    if (target_s <= current_s + 1.0e-3) {
      return std::nullopt;
    }
    waypoints.emplace_back(target_s, offsets[i]);
    current_s = target_s;
    current_d = offsets[i];
    const double hold_end = std::clamp(
      clusters[i].max_s + parameters_.obstacle_longitudinal_margin_m,
      current_s + parameters_.path_sampling_interval_m, candidate_end_s);
    if (hold_end > current_s + 1.0e-3) {
      waypoints.emplace_back(hold_end, offsets[i]);
      current_s = hold_end;
    }
  }
  const double return_s =
    std::min(candidate_end_s, current_s + parameters_.shift_return_distance_m);
  if (return_s > current_s + 1.0e-3) {
    waypoints.emplace_back(return_s, 0.0);
    current_s = return_s;
  }
  if (candidate_end_s > current_s + 1.0e-3) {
    waypoints.emplace_back(candidate_end_s, 0.0);
  }

  std::vector<autoware::sampler_common::FrenetPoint> frenet_points;
  autoware::frenet_planner::FrenetState start;
  start.position = {waypoints.front().first, waypoints.front().second};
  for (size_t i = 1; i < waypoints.size(); ++i) {
    autoware::frenet_planner::FrenetState target;
    target.position = {waypoints[i].first, waypoints[i].second};
    auto segment = autoware::frenet_planner::generateCandidate(
      start, target, parameters_.path_sampling_interval_m);
    if (segment.frenet_points.size() < 2) {
      return std::nullopt;
    }
    const size_t skip = frenet_points.empty() ? 0U : 1U;
    frenet_points.insert(
      frenet_points.end(),
      std::next(segment.frenet_points.begin(), static_cast<std::ptrdiff_t>(skip)),
      segment.frenet_points.end());
    start = target;
  }
  if (frenet_points.size() < 2) {
    return std::nullopt;
  }

  Trajectory output;
  output.header = reference_trajectory.header;
  output.points.reserve(frenet_points.size());
  for (const auto & frenet : frenet_points) {
    const auto cartesian = reference.cartesian(frenet);
    TrajectoryPoint point;
    point.pose.position.x = cartesian.x();
    point.pose.position.y = cartesian.y();
    point.pose.position.z =
      interpolate_value(reference_s, reference_trajectory.points, frenet.s, false);
    point.longitudinal_velocity_mps = static_cast<float>(
      interpolate_value(reference_s, reference_trajectory.points, frenet.s, true));
    output.points.push_back(point);
  }
  for (size_t i = 0; i < output.points.size(); ++i) {
    const size_t previous = i == 0 ? 0 : i - 1;
    const size_t next = std::min(i + 1, output.points.size() - 1);
    const auto & from = output.points[previous].pose.position;
    const auto & to = output.points[next].pose.position;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, std::atan2(to.y - from.y, to.x - from.x));
    output.points[i].pose.orientation = tf2::toMsg(orientation);
  }
  return output;
}

std::optional<CandidateTrajectories> FixedRouteObstacleBypassPlannerNode::smooth_candidate(
  Trajectory candidate, const VelocityProfile profile, const nav_msgs::msg::Odometry & odometry,
  const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration,
  const std::optional<autoware_internal_planning_msgs::msg::VelocityLimit> &
    external_velocity_limit) const
{
  if (candidate.points.size() < 2) {
    return std::nullopt;
  }
  for (auto & point : candidate.points) {
    // make_geometric_candidate interpolates velocity by the reference path's
    // arc length. Keep that value as the regulatory upper bound: a nearest
    // Cartesian lookup can select a different branch on loops or parallel
    // lanelets and accidentally erase a signal/stop-line cap.
    const double regulatory_cap =
      std::max(0.0, static_cast<double>(point.longitudinal_velocity_mps));
    double profile_cap = regulatory_cap;
    switch (profile) {
      case VelocityProfile::ACCELERATED:
        profile_cap = regulatory_cap * std::min(1.0, parameters_.accelerated_velocity_ratio);
        break;
      case VelocityProfile::NORMAL:
        profile_cap = std::min(regulatory_cap, parameters_.normal_target_velocity_mps);
        break;
      case VelocityProfile::REDUCED:
        profile_cap = std::min(
          regulatory_cap, std::max(
                            parameters_.creep_velocity_mps, parameters_.normal_target_velocity_mps *
                                                              parameters_.reduced_velocity_ratio));
        break;
      case VelocityProfile::CREEP:
        profile_cap = std::min(regulatory_cap, parameters_.creep_velocity_mps);
        break;
    }
    if (external_velocity_limit) {
      profile_cap =
        std::min(profile_cap, static_cast<double>(external_velocity_limit->max_velocity));
    }
    point.longitudinal_velocity_mps = static_cast<float>(std::max(0.0, profile_cap));
  }

  const double measured_velocity = std::max(0.0, odometry.twist.twist.linear.x);
  const double measured_acceleration = acceleration.accel.accel.linear.x;
  double smoothing_initial_velocity = measured_velocity;
  double smoothing_initial_acceleration = measured_acceleration;

  // VelocitySmootherNode does not always initialize its optimizer from the
  // measured standstill velocity.  Once it has a previous output, it uses the
  // configured engage velocity/acceleration while the vehicle is below the
  // engage threshold and no stop is immediately ahead.  Mirror that contract
  // here; calling JerkFilteredSmoother::apply() directly with v0=0 on every
  // cycle otherwise predicts a long near-zero prefix that the downstream node
  // never publishes.
  const auto target_ego_index = nearest_trajectory_index(
    candidate.points, odometry.pose.pose, parameters_.nearest_distance_threshold_m,
    parameters_.nearest_yaw_threshold_rad);
  if (!target_ego_index) {
    return std::nullopt;
  }
  const auto target_arc = trajectory_arc_lengths(candidate.points);
  double stop_distance = std::numeric_limits<double>::infinity();
  for (size_t i = *target_ego_index; i + 1 < candidate.points.size(); ++i) {
    if (
      std::abs(candidate.points[i].longitudinal_velocity_mps) <=
      parameters_.zero_velocity_epsilon_mps) {
      stop_distance = target_arc[i] - target_arc[*target_ego_index];
      break;
    }
  }
  const double target_velocity = std::abs(
    static_cast<double>(candidate.points[*target_ego_index].longitudinal_velocity_mps));
  const double engage_threshold =
    parameters_.smoother_engage_velocity_mps * parameters_.smoother_engage_exit_ratio;
  const bool use_engage_initial_condition =
    measured_velocity < engage_threshold &&
    target_velocity >= parameters_.smoother_engage_velocity_mps &&
    stop_distance > parameters_.smoother_stop_distance_to_prohibit_engage_m;
  if (use_engage_initial_condition) {
    smoothing_initial_velocity = parameters_.smoother_engage_velocity_mps;
    smoothing_initial_acceleration = parameters_.smoother_engage_acceleration_mps2;
  }

  auto filtered = smoother_->resampleTrajectory(
    candidate.points, measured_velocity, odometry.pose.pose,
    parameters_.nearest_distance_threshold_m, parameters_.nearest_yaw_threshold_rad);
  if (filtered.size() < 2) {
    return std::nullopt;
  }
  filtered = smoother_->applyLateralAccelerationFilter(
    filtered, smoothing_initial_velocity, smoothing_initial_acceleration, true, false,
    parameters_.path_sampling_interval_m);
  filtered =
    smoother_->applySteeringRateLimit(filtered, false, parameters_.path_sampling_interval_m);

  // The rear part of the candidate is useful to the geometric filters, but the
  // longitudinal smoother must receive its initial conditions at the current
  // ego projection rather than at the rear-buffer point.
  Trajectory filtered_trajectory = candidate;
  filtered_trajectory.points = std::move(filtered);
  const auto ego_aligned_input = align_trajectory_to_ego(
    filtered_trajectory, odometry.pose.pose, parameters_.planning_horizon_m,
    parameters_.nearest_distance_threshold_m, parameters_.nearest_yaw_threshold_rad);
  if (!ego_aligned_input) {
    return std::nullopt;
  }
  // This is the command-side contract. Its velocities are desired/regulatory
  // upper bounds, not the velocity that the vehicle is predicted to realize
  // from its current state. In particular, do not write the measured startup
  // velocity into this trajectory: the downstream Autoware velocity smoother
  // interprets any near-zero input velocity as an intentional stop point.
  Trajectory downstream_target = *ego_aligned_input;

  std::vector<TrajectoryPoint> output_points;
  std::vector<std::vector<TrajectoryPoint>> debug_trajectories;
  if (
    !smoother_->apply(
      smoothing_initial_velocity, smoothing_initial_acceleration, downstream_target.points,
      output_points, debug_trajectories, false) ||
    output_points.size() < 2) {
    return std::nullopt;
  }
  candidate.points = std::move(output_points);
  const auto ego_aligned_output = align_trajectory_to_ego(
    candidate, odometry.pose.pose, parameters_.planning_horizon_m,
    parameters_.nearest_distance_threshold_m, parameters_.nearest_yaw_threshold_rad);
  if (!ego_aligned_output) {
    return std::nullopt;
  }
  candidate = *ego_aligned_output;
  candidate.points.front().longitudinal_velocity_mps =
    static_cast<float>(smoothing_initial_velocity);
  candidate.points.front().acceleration_mps2 =
    static_cast<float>(smoothing_initial_acceleration);
  assign_time_from_start(
    candidate.points, measured_velocity, parameters_.max_longitudinal_acceleration_mps2);
  return CandidateTrajectories{std::move(downstream_target), std::move(candidate)};
}

void FixedRouteObstacleBypassPlannerNode::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  const auto callback_started = std::chrono::steady_clock::now();
  autoware_utils_debug::ScopedTimeTrack time_track("fixed_route_bypass", *time_keeper_);
  const auto planning_time = now();

  std::optional<nav_msgs::msg::Odometry> odometry;
  std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> acceleration;
  std::optional<autoware_internal_planning_msgs::msg::VelocityLimit> external_velocity_limit;
  ObjectTracker object_tracker_snapshot{parameters_};
  std::optional<Corridor> corridor;
  std::optional<Trajectory> last_valid_target;
  std::optional<Trajectory> last_valid_prediction;
  VelocityProfile last_valid_velocity_profile = VelocityProfile::NORMAL;
  rclcpp::Time last_valid_time{0, 0, RCL_ROS_TIME};
  uint64_t last_valid_route_generation = 0;
  PassSide committed_side = PassSide::CENTER;
  std::vector<std::string> committed_objects;
  std::string input_error;
  CandidateFailureReason input_failure = CandidateFailureReason::NONE;
  {
    // Input callbacks run in a separate callback group. Hold the mutex only
    // while making a coherent planning-cycle snapshot; the expensive
    // smoothing and polygon validation below must not prevent objects,
    // odometry, or acceleration from being refreshed.
    std::lock_guard<std::mutex> lock(mutex_);
    object_tracker_.prune(planning_time);
    if (odometry_) {
      odometry = *odometry_;
    }
    if (acceleration_) {
      acceleration = *acceleration_;
    }
    if (external_velocity_limit_) {
      external_velocity_limit = *external_velocity_limit_;
    }
    const bool object_stream_fresh =
      objects_received_ &&
      (planning_time - last_objects_received_time_).seconds() <=
        std::max(parameters_.object_dropout_ttl_s, parameters_.max_input_stamp_skew_s);
    if (!odometry || !acceleration || !object_stream_fresh || !corridor_builder_.ready()) {
      input_error = "route/path/map/objects/kinematics not ready";
    } else if (!corridor_builder_.input_is_current(
                 msg->header.stamp, parameters_.max_input_stamp_skew_s, &input_error)) {
      // input_error is populated by the corridor builder.
    } else {
      const double vehicle_half_width = std::max(
        std::abs(vehicle_info_.min_lateral_offset_m),
        std::abs(vehicle_info_.max_lateral_offset_m));
      corridor = corridor_builder_.build(
        odometry->pose.pose, parameters_.planning_horizon_m, vehicle_half_width, parameters_,
        &input_error, &input_failure);
      if (corridor) {
        object_tracker_snapshot = object_tracker_;
        last_valid_target = last_valid_target_;
        last_valid_prediction = last_valid_prediction_;
        last_valid_velocity_profile = last_valid_velocity_profile_;
        last_valid_time = last_valid_time_;
        last_valid_route_generation = last_valid_route_generation_;
        committed_side = committed_side_;
        committed_objects = committed_objects_;
      }
    }
  }
  const auto snapshot_completed = std::chrono::steady_clock::now();

  const auto handle_input_failure = [&](
                                      const std::string & reason,
                                      const CandidateFailureReason failure_reason =
                                        CandidateFailureReason::NONE) {
    std::map<CandidateFailureReason, size_t> failure_counts;
    CandidateFailureContext failure_context;
    const CandidateFailureContext * failure_context_ptr = nullptr;
    if (failure_reason != CandidateFailureReason::NONE) {
      failure_counts[failure_reason] = 1;
      failure_context.reason = failure_reason;
      failure_context_ptr = &failure_context;
    }
    if (odometry && msg->points.size() >= 2) {
      auto emergency = make_emergency_stop_trajectory(*msg, *odometry, planning_time, parameters_);
      publish_state(
        PlanningMode::EMERGENCY, "planner input failure: " + reason, 0, nullptr,
        failure_counts.empty() ? nullptr : &failure_counts, failure_context_ptr, nullptr,
        emergency.header.stamp);
      trajectory_pub_->publish(emergency);
    } else {
      publish_state(
        PlanningMode::WAITING_FOR_INPUT, reason, 0, nullptr,
        failure_counts.empty() ? nullptr : &failure_counts, failure_context_ptr, nullptr,
        msg->header.stamp);
    }
  };
  if (!input_error.empty() || !odometry || !acceleration || !corridor) {
    handle_input_failure(
      input_error.empty() ? "route/path/map/objects/kinematics not ready" : input_error,
      input_failure);
    return;
  }
  if (msg->points.size() < 2) {
    handle_input_failure(
      "optimized trajectory is empty", CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY);
    return;
  }

  std::vector<TrajectoryPoint> reference_points;
  reference_points.reserve(msg->points.size());
  for (const auto & point : msg->points) {
    if (
      reference_points.empty() ||
      point_distance(reference_points.back().pose.position, point.pose.position) > 0.05) {
      reference_points.push_back(point);
    }
  }
  if (reference_points.size() < 2) {
    handle_input_failure(
      "optimized trajectory has duplicate geometry",
      CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY);
    return;
  }
  Trajectory reference_trajectory = *msg;
  reference_trajectory.points = reference_points;
  const auto reference_s = trajectory_arc_lengths(reference_points);
  const auto occupancy_timeline =
    make_occupancy_timeline(object_tracker_snapshot, planning_time, parameters_);

  // A previously selected trajectory is already the result of the full
  // lateral/profile search.  Reconnect it to the current ego pose, apply the
  // newest regulatory velocity upper bounds, and re-run the complete current
  // corridor and swept-occupancy validation before considering a new search.
  // This is the normal receding-horizon contract: a new object, route/path
  // change, or invalid remaining geometry immediately falls through to the
  // exhaustive candidate generator below.
  if (
    last_valid_target && last_valid_prediction &&
    last_valid_route_generation == corridor->route_generation) {
    auto remaining_target =
      crop_downstream_target(*last_valid_target, *odometry, planning_time, parameters_);
    const auto current_reference = align_trajectory_to_ego(
      reference_trajectory, odometry->pose.pose, parameters_.planning_horizon_m,
      parameters_.nearest_distance_threshold_m, parameters_.nearest_yaw_threshold_rad);
    if (remaining_target && current_reference) {
      const auto remaining_arc = trajectory_arc_lengths(remaining_target->points);
      const auto current_reference_arc = trajectory_arc_lengths(current_reference->points);
      for (size_t i = 0; i < remaining_target->points.size(); ++i) {
        const double regulatory_cap = std::max(
          0.0, interpolate_value(
                 current_reference_arc, current_reference->points, remaining_arc[i], true));
        remaining_target->points[i].longitudinal_velocity_mps = static_cast<float>(std::min(
          std::max(
            0.0,
            static_cast<double>(remaining_target->points[i].longitudinal_velocity_mps)),
          regulatory_cap));
      }
      const auto revalidated_candidate = smooth_candidate(
        *remaining_target, last_valid_velocity_profile, *odometry, *acceleration,
        external_velocity_limit);
      if (revalidated_candidate) {
        const auto & prediction = revalidated_candidate->prediction;
        const auto startup_validation = check_unexpected_zero_velocity(
          prediction, reference_trajectory, odometry->twist.twist.linear.x, parameters_);
        const bool progress_valid =
          has_nonterminal_regulatory_stop(
            reference_trajectory, prediction.points.front().pose, parameters_) ||
          progress_at_time(prediction, parameters_.progress_horizon_s) >=
            parameters_.minimum_progress_m;
        const auto validation = validate_trajectory(
          prediction, *corridor, object_tracker_snapshot, planning_time, vehicle_info_, parameters_,
          true, true, false, true, true, &occupancy_timeline);
        if (startup_validation.valid && progress_valid && validation.valid) {
          bool route_is_current = false;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            route_is_current = corridor_builder_.route_generation() == corridor->route_generation;
            if (route_is_current) {
              last_valid_target_ = revalidated_candidate->downstream_target;
              last_valid_prediction_ = prediction;
              last_valid_velocity_profile_ = last_valid_velocity_profile;
              last_valid_time_ = planning_time;
              last_valid_route_generation_ = corridor->route_generation;
              committed_side_ = committed_side;
              committed_objects_ = committed_objects;
            }
          }
          if (!route_is_current) {
            handle_input_failure("active route changed while revalidating the selected trajectory");
            return;
          }
          CandidateDescriptor descriptor;
          descriptor.velocity_profile = last_valid_velocity_profile;
          descriptor.pass_side = committed_side;
          descriptor.minimum_clearance_m = validation.minimum_clearance_m;
          publish_state(
            PlanningMode::LAST_VALID, "revalidated selected trajectory", 1, &descriptor, nullptr,
            nullptr, &*corridor, revalidated_candidate->downstream_target.header.stamp);
          trajectory_pub_->publish(revalidated_candidate->downstream_target);
          publish_markers(*corridor, {}, &prediction);
          const auto revalidation_completed = std::chrono::steady_clock::now();
          const auto milliseconds = [](const auto begin, const auto end) {
            return std::chrono::duration<double, std::milli>(end - begin).count();
          };
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "planner revalidation timing [ms]: snapshot=%.1f revalidation=%.1f total=%.1f",
            milliseconds(callback_started, snapshot_completed),
            milliseconds(snapshot_completed, revalidation_completed),
            milliseconds(callback_started, revalidation_completed));
          return;
        }
        RCLCPP_WARN(
          get_logger(),
          "selected trajectory revalidation failed: startup=%s progress=%s "
          "validation=%s failure_index=%zu failure_time=%.3f",
          to_string(startup_validation.failure_reason), progress_valid ? "valid" : "insufficient",
          validation.reason.c_str(), validation.trajectory_index, validation.trajectory_time_s);
      } else {
        RCLCPP_WARN(get_logger(), "selected trajectory revalidation failed: smoother failure");
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "selected trajectory revalidation failed: remaining_target=%s current_reference=%s",
        remaining_target ? "valid" : "invalid", current_reference ? "valid" : "invalid");
    }
  }

  // Exhaustive geometry/profile search can take longer than the planning
  // topic's watchdog period. A separate callback publishes a fresh emergency
  // trajectory from the latest Ego pose while no normal trajectory has yet
  // been certified. This preserves both the safety contract and output
  // liveness; it never republishes an unvalidated stale bypass path.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    exhaustive_search_in_progress_ = true;
    search_guard_reference_ = reference_trajectory;
  }
  const auto finish_exhaustive_search = [this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    exhaustive_search_in_progress_ = false;
    search_guard_reference_.reset();
  };

  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(reference_points.size());
  ys.reserve(reference_points.size());
  for (const auto & point : reference_points) {
    xs.push_back(point.pose.position.x);
    ys.push_back(point.pose.position.y);
  }
  const Spline2D reference(xs, ys);
  const auto ego_frenet = reference.frenet(
    Point2d{odometry->pose.pose.position.x, odometry->pose.pose.position.y});
  const auto clusters = make_obstacle_clusters(
    reference, reference_points, reference_s, object_tracker_snapshot, occupancy_timeline,
    planning_time, ego_frenet.s);
  const auto clusters_completed = std::chrono::steady_clock::now();

  if (!committed_objects.empty()) {
    const auto still_present =
      std::any_of(clusters.begin(), clusters.end(), [&](const auto & cluster) {
        return std::any_of(
          cluster.object_ids.begin(), cluster.object_ids.end(), [&](const auto & id) {
            return std::find(committed_objects.begin(), committed_objects.end(), id) !=
                   committed_objects.end();
          });
      });
    if (!still_present) {
      committed_objects.clear();
      committed_side = PassSide::CENTER;
    }
  }

  std::optional<CandidateTrajectories> selected_candidate;
  CandidateDescriptor selected_descriptor;
  size_t valid_candidate_count = 0;
  size_t generated_geometry_count = 0;
  std::map<CandidateFailureReason, size_t> failure_counts;
  std::optional<CandidateFailureContext> first_failure;
  const ObjectTracker empty_object_tracker{parameters_};
  double point_containment_time_ms = 0.0;
  double swept_validation_time_ms = 0.0;
  size_t validation_count = 0;
  const auto record_failure = [&failure_counts,
                               &first_failure](const CandidateFailureContext & failure) {
    ++failure_counts[failure.reason];
    if (!first_failure) {
      first_failure = failure;
    }
  };
  const auto assignments = make_lateral_assignments(clusters.size());
  const std::array profiles{
    VelocityProfile::ACCELERATED, VelocityProfile::NORMAL, VelocityProfile::REDUCED,
    VelocityProfile::CREEP};
  for (const auto & offsets : assignments) {
    const PassSide side = side_from_offsets(offsets);
    const auto geometry = make_geometric_candidate(
      reference_trajectory, reference, reference_s, ego_frenet.s, ego_frenet.d, clusters, offsets);
    if (!geometry) {
      continue;
    }
    ++generated_geometry_count;
    // Corridor containment depends only on geometry. Check it once before
    // running four longitudinal smoothers; otherwise the same out-of-corridor
    // path is smoothed and rejected once per velocity profile.
    const auto aligned_geometry = align_trajectory_to_ego(
      *geometry, odometry->pose.pose, parameters_.planning_horizon_m,
      parameters_.nearest_distance_threshold_m, parameters_.nearest_yaw_threshold_rad);
    if (!aligned_geometry) {
      CandidateFailureContext failure;
      failure.reason = CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY;
      failure.pass_side = side;
      record_failure(failure);
      continue;
    }
    std::optional<size_t> center_space_failure_index;
    for (size_t i = 0; i < aligned_geometry->points.size(); ++i) {
      const Point2d center{
        aligned_geometry->points[i].pose.position.x,
        aligned_geometry->points[i].pose.position.y};
      if (!boost::geometry::covered_by(center, corridor->center_space)) {
        center_space_failure_index = i;
        break;
      }
    }
    if (center_space_failure_index) {
      CandidateFailureContext failure;
      failure.reason = CandidateFailureReason::TRAJECTORY_OUTSIDE_CORRIDOR;
      failure.pass_side = side;
      failure.trajectory_index = *center_space_failure_index;
      const auto arc = trajectory_arc_lengths(aligned_geometry->points);
      if (*center_space_failure_index < arc.size()) {
        failure.trajectory_s = arc[*center_space_failure_index];
      }
      record_failure(failure);
      continue;
    }
    // Full footprint and swept-route containment are properties of the path
    // geometry, not of ACCELERATED/NORMAL/REDUCED/CREEP timing.  Validate
    // them once per geometry and leave only timed occupancy and kinematic
    // checks to each longitudinal profile.
    const auto spatial_validation = validate_trajectory(
      *aligned_geometry, *corridor, empty_object_tracker, planning_time, vehicle_info_,
      parameters_, false, true, false, true, false);
    ++validation_count;
    point_containment_time_ms += spatial_validation.point_containment_time_ms;
    swept_validation_time_ms += spatial_validation.swept_validation_time_ms;
    if (!spatial_validation.valid) {
      CandidateFailureContext failure;
      failure.reason = spatial_validation.failure_reason;
      failure.pass_side = side;
      failure.trajectory_index = spatial_validation.trajectory_index;
      const auto arc = trajectory_arc_lengths(aligned_geometry->points);
      if (spatial_validation.trajectory_index < arc.size()) {
        failure.trajectory_s = arc[spatial_validation.trajectory_index];
      }
      record_failure(failure);
      continue;
    }
    for (const auto profile : profiles) {
      const auto candidate =
        smooth_candidate(*geometry, profile, *odometry, *acceleration, external_velocity_limit);
      if (!candidate) {
        CandidateFailureContext failure;
        failure.reason = CandidateFailureReason::SMOOTHER_FAILURE;
        failure.velocity_profile = profile;
        failure.pass_side = side;
        record_failure(failure);
        continue;
      }
      const auto & prediction = candidate->prediction;
      const auto startup_validation = check_unexpected_zero_velocity(
        prediction, reference_trajectory, odometry->twist.twist.linear.x, parameters_);
      if (!startup_validation.valid) {
        CandidateFailureContext failure;
        failure.reason = startup_validation.failure_reason;
        failure.velocity_profile = profile;
        failure.pass_side = side;
        failure.trajectory_index = startup_validation.failure_index;
        failure.trajectory_s = startup_validation.failure_s;
        failure.velocity_mps = startup_validation.velocity_mps;
        failure.reference_velocity_mps = startup_validation.reference_velocity_mps;
        failure.startup_release_index = startup_validation.startup_release_index;
        failure.startup_release_time_s = startup_validation.startup_release_time_s;
        record_failure(failure);
        continue;
      }
      const double candidate_progress =
        progress_at_time(prediction, parameters_.progress_horizon_s);
      if (
        !has_nonterminal_regulatory_stop(
          reference_trajectory, prediction.points.front().pose, parameters_) &&
        candidate_progress < parameters_.minimum_progress_m) {
        CandidateFailureContext failure;
        failure.reason = CandidateFailureReason::INSUFFICIENT_PROGRESS;
        failure.velocity_profile = profile;
        failure.pass_side = side;
        failure.trajectory_s = candidate_progress;
        failure.startup_release_index = startup_validation.startup_release_index;
        record_failure(failure);
        continue;
      }
      // JerkFilteredSmoother is the longitudinal source of truth for this
      // prediction. Its acceleration/jerk constraints are soft optimization
      // constraints, so re-differentiating the sampled result and applying a
      // second hard jerk threshold can intermittently reject the smoother's
      // own valid output. Keep the independent lateral/geometric, corridor,
      // progress, and swept-collision checks here.
      const auto validation = validate_trajectory(
        prediction, *corridor, object_tracker_snapshot, planning_time, vehicle_info_, parameters_,
        true, false, false, false, true, &occupancy_timeline);
      ++validation_count;
      point_containment_time_ms += validation.point_containment_time_ms;
      swept_validation_time_ms += validation.swept_validation_time_ms;
      if (!validation.valid) {
        CandidateFailureContext failure;
        failure.reason = validation.failure_reason;
        failure.velocity_profile = profile;
        failure.pass_side = side;
        failure.trajectory_index = validation.trajectory_index;
        failure.trajectory_time_s = validation.trajectory_time_s;
        failure.object_id = validation.object_id;
        const auto arc = trajectory_arc_lengths(prediction.points);
        if (validation.trajectory_index < arc.size()) {
          failure.trajectory_s = arc[validation.trajectory_index];
          failure.velocity_mps =
            std::abs(prediction.points[validation.trajectory_index].longitudinal_velocity_mps);
        }
        failure.startup_release_index = startup_validation.startup_release_index;
        if (
          side == PassSide::CENTER &&
          validation.failure_reason == CandidateFailureReason::COLLISION_FAILURE) {
          RCLCPP_WARN(
            get_logger(),
            "center profile rejected: profile=%s object=%s s=%.3f t=%.3f index=%zu "
            "ego_box=[%.2f,%.2f]-[%.2f,%.2f] object_box=[%.2f,%.2f]-[%.2f,%.2f]",
            to_string(profile), validation.object_id.c_str(), failure.trajectory_s,
            validation.trajectory_time_s, validation.trajectory_index,
            validation.ego_envelope_min_x, validation.ego_envelope_min_y,
            validation.ego_envelope_max_x, validation.ego_envelope_max_y,
            validation.object_envelope_min_x, validation.object_envelope_min_y,
            validation.object_envelope_max_x, validation.object_envelope_max_y);
        }
        record_failure(failure);
        continue;
      }
      ++valid_candidate_count;
      const double lateral_cost = std::accumulate(
        offsets.begin(), offsets.end(), 0.0,
        [](const double sum, const double offset) { return sum + std::abs(offset); });
      const double progress = progress_at_time(prediction, parameters_.progress_horizon_s);
      const double clearance_reward = std::isfinite(validation.minimum_clearance_m)
                                        ? std::min(validation.minimum_clearance_m, 5.0)
                                        : 5.0;
      const double score = lateral_cost - 2.0 * progress - 0.1 * clearance_reward;
      // Commitment prevents nominal left/right oscillation, but it must not
      // discard every safe recovery when the committed geometry itself has
      // become invalid. Keep a large preference for the committed side and
      // select another side only when no committed candidate validates.
      const double commitment_penalty =
        committed_side != PassSide::CENTER && side != committed_side ? 1000.0 : 0.0;
      const double committed_score = score + commitment_penalty;
      if (!selected_candidate || committed_score < selected_descriptor.score) {
        selected_candidate = *candidate;
        selected_descriptor.cluster_offsets = offsets;
        selected_descriptor.velocity_profile = profile;
        selected_descriptor.pass_side = side;
        selected_descriptor.score = committed_score;
        selected_descriptor.minimum_clearance_m = validation.minimum_clearance_m;
      }
    }
  }

  if (selected_candidate) {
    const auto search_completed = std::chrono::steady_clock::now();
    selected_candidate->downstream_target.header.stamp = msg->header.stamp;
    selected_candidate->prediction.header.stamp = msg->header.stamp;
    if (!clusters.empty() && selected_descriptor.pass_side != PassSide::CENTER) {
      committed_side = selected_descriptor.pass_side;
      committed_objects.clear();
      for (const auto & cluster : clusters) {
        committed_objects.insert(
          committed_objects.end(), cluster.object_ids.begin(), cluster.object_ids.end());
      }
    }
    bool route_is_current = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      route_is_current = corridor_builder_.route_generation() == corridor->route_generation;
      if (route_is_current) {
        last_valid_target_ = selected_candidate->downstream_target;
        last_valid_prediction_ = selected_candidate->prediction;
        last_valid_velocity_profile_ = selected_descriptor.velocity_profile;
        last_valid_time_ = planning_time;
        last_valid_route_generation_ = corridor->route_generation;
        committed_side_ = committed_side;
        committed_objects_ = committed_objects;
      }
    }
    if (!route_is_current) {
      finish_exhaustive_search();
      handle_input_failure("active route changed while planning");
      return;
    }
    finish_exhaustive_search();
    publish_state(
      clusters.empty() ? PlanningMode::NORMAL : PlanningMode::NORMAL_BYPASS, "valid candidate",
      valid_candidate_count, &selected_descriptor, &failure_counts,
      first_failure ? &*first_failure : nullptr, &*corridor,
      selected_candidate->downstream_target.header.stamp);
    trajectory_pub_->publish(selected_candidate->downstream_target);
    publish_markers(*corridor, clusters, &selected_candidate->prediction);
    const auto milliseconds = [](const auto begin, const auto end) {
      return std::chrono::duration<double, std::milli>(end - begin).count();
    };
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "planner timing [ms]: snapshot=%.1f reference_and_clusters=%.1f search=%.1f "
      "containment_sum=%.1f swept_sum=%.1f validations=%zu geometries=%zu total=%.1f",
      milliseconds(callback_started, snapshot_completed),
      milliseconds(snapshot_completed, clusters_completed),
      milliseconds(clusters_completed, search_completed), point_containment_time_ms,
      swept_validation_time_ms, validation_count, generated_geometry_count,
      milliseconds(callback_started, search_completed));
    return;
  }

  if (
    last_valid_target && last_valid_prediction &&
    last_valid_route_generation == corridor->route_generation &&
    (planning_time - last_valid_time).seconds() <= parameters_.last_valid_hold_time_s) {
    const auto remaining_target =
      crop_downstream_target(*last_valid_target, *odometry, planning_time, parameters_);
    const auto remaining_candidate = remaining_target
                                       ? smooth_candidate(
                                           *remaining_target, last_valid_velocity_profile,
                                           *odometry, *acceleration, external_velocity_limit)
                                       : std::nullopt;
    if (remaining_candidate) {
      const auto & remaining_prediction = remaining_candidate->prediction;
      const auto startup_validation = check_unexpected_zero_velocity(
        remaining_prediction, reference_trajectory, odometry->twist.twist.linear.x, parameters_);
      const bool progress_valid =
        has_nonterminal_regulatory_stop(
          reference_trajectory, remaining_prediction.points.front().pose, parameters_) ||
        progress_at_time(remaining_prediction, parameters_.progress_horizon_s) >=
          parameters_.minimum_progress_m;
      const auto validation = validate_trajectory(
        remaining_prediction, *corridor, object_tracker_snapshot, planning_time, vehicle_info_,
        parameters_, true, true, false, true, true, &occupancy_timeline);
      if (startup_validation.valid && progress_valid && validation.valid) {
        bool route_is_current = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          route_is_current = corridor_builder_.route_generation() == corridor->route_generation;
          if (route_is_current) {
            last_valid_prediction_ = remaining_prediction;
            committed_side_ = committed_side;
            committed_objects_ = committed_objects;
          }
        }
        if (!route_is_current) {
          finish_exhaustive_search();
          handle_input_failure("active route changed while revalidating the last trajectory");
          return;
        }
        finish_exhaustive_search();
        publish_state(
          PlanningMode::LAST_VALID, "new candidate infeasible; revalidated last valid trajectory",
          0, nullptr, &failure_counts, first_failure ? &*first_failure : nullptr, &*corridor,
          remaining_candidate->downstream_target.header.stamp);
        trajectory_pub_->publish(remaining_candidate->downstream_target);
        publish_markers(*corridor, clusters, &remaining_prediction);
        return;
      }
    }
  }

  std::string failure = "NO_FEASIBLE_NONSTOP_TRAJECTORY";
  if (generated_geometry_count == 0) {
    failure_counts[CandidateFailureReason::NO_LATERAL_SOLUTION] = 1;
  }
  for (const auto & [reason, count] : failure_counts) {
    if (reason != CandidateFailureReason::NONE && count > 0) {
      failure += ":" + std::string(to_string(reason));
    }
  }
  auto emergency = make_emergency_stop_trajectory(*msg, *odometry, planning_time, parameters_);
  finish_exhaustive_search();
  publish_state(
    PlanningMode::EMERGENCY, failure + "; no current or revalidated last-valid path is safe", 0,
    nullptr, &failure_counts, first_failure ? &*first_failure : nullptr, &*corridor,
    emergency.header.stamp);
  trajectory_pub_->publish(emergency);
  publish_markers(*corridor, clusters, &emergency);
}

void FixedRouteObstacleBypassPlannerNode::on_search_guard_timer()
{
  // Keep the mutex through publication so finish_exhaustive_search() cannot
  // clear the flag and publish a normal result while an already-started timer
  // callback subsequently overwrites it with an emergency message.
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !exhaustive_search_in_progress_ || !search_guard_reference_ || !odometry_ ||
    search_guard_reference_->points.size() < 2) {
    return;
  }

  const auto planning_time = now();
  auto emergency = make_emergency_stop_trajectory(
    *search_guard_reference_, *odometry_, planning_time, parameters_);
  publish_state(
    PlanningMode::EMERGENCY, "exhaustive candidate search in progress; fresh emergency guard", 0,
    nullptr, nullptr, nullptr, nullptr, emergency.header.stamp);
  trajectory_pub_->publish(emergency);
}

void FixedRouteObstacleBypassPlannerNode::publish_state(
  const PlanningMode mode, const std::string & reason, const size_t candidate_count,
  const CandidateDescriptor * selected,
  const std::map<CandidateFailureReason, size_t> * failure_counts,
  const CandidateFailureContext * failure_context, const Corridor * corridor,
  const builtin_interfaces::msg::Time & trajectory_stamp)
{
  autoware_internal_debug_msgs::msg::StringStamped state;
  state.stamp = trajectory_stamp;
  state.data = to_string(mode);
  state_pub_->publish(state);

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "planning/fixed_route_obstacle_bypass";
  status.hardware_id = "planning";
  status.message = reason;
  status.level = mode == PlanningMode::EMERGENCY
                   ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                   : (mode == PlanningMode::LAST_VALID || mode == PlanningMode::WAITING_FOR_INPUT
                        ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                        : diagnostic_msgs::msg::DiagnosticStatus::OK);
  status.values.push_back(key_value("mode", to_string(mode)));
  status.values.push_back(key_value("valid_candidate_count", std::to_string(candidate_count)));
  status.values.push_back(key_value("route_uuid", corridor ? corridor->route_uuid : std::string{}));
  if (selected) {
    status.values.push_back(key_value("pass_side", to_string(selected->pass_side)));
    status.values.push_back(key_value("velocity_profile", to_string(selected->velocity_profile)));
    status.values.push_back(key_value("score", std::to_string(selected->score)));
  }
  if (corridor) {
    status.values.push_back(key_value("ego_index", "0"));
    status.values.push_back(
      key_value("corridor_lane_ids", lane_ids_to_string(corridor->ordered_lane_ids)));
    status.values.push_back(
      key_value("path_lane_ids_at_ego", lane_ids_to_string(corridor->ego_path_lane_ids)));
    status.values.push_back(key_value("ego_lane_id", std::to_string(corridor->ego_lane_id)));
  }
  if (failure_counts) {
    for (const auto & [failure_reason, count] : *failure_counts) {
      status.values.push_back(key_value(
        "failure_count." + std::string(to_string(failure_reason)), std::to_string(count)));
    }
  }
  if (failure_context) {
    status.values.push_back(key_value("failure_object_id", failure_context->object_id));
    status.values.push_back(key_value("failure_reason", to_string(failure_context->reason)));
    status.values.push_back(key_value("failure_path_type", to_string(failure_context->pass_side)));
    status.values.push_back(
      key_value("failure_velocity_profile", to_string(failure_context->velocity_profile)));
    status.values.push_back(
      key_value("failure_index", std::to_string(failure_context->trajectory_index)));
    status.values.push_back(key_value("failure_s", std::to_string(failure_context->trajectory_s)));
    status.values.push_back(
      key_value("failure_time_s", std::to_string(failure_context->trajectory_time_s)));
    status.values.push_back(
      key_value("failure_velocity", std::to_string(failure_context->velocity_mps)));
    status.values.push_back(key_value(
      "failure_reference_velocity", std::to_string(failure_context->reference_velocity_mps)));
    status.values.push_back(key_value(
      "startup_release_index", failure_context->startup_release_index
                                 ? std::to_string(*failure_context->startup_release_index)
                                 : std::string{"NONE"}));
    status.values.push_back(key_value(
      "startup_release_time_s", std::to_string(failure_context->startup_release_time_s)));
  }
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);

  if (parameters_.debug_logging) {
    RCLCPP_INFO(
      get_logger(), "mode=%s reason=%s candidates=%zu side=%s velocity_profile=%s", to_string(mode),
      reason.c_str(), candidate_count, selected ? to_string(selected->pass_side) : "NONE",
      selected ? to_string(selected->velocity_profile) : "NONE");
  }
}

void FixedRouteObstacleBypassPlannerNode::publish_markers(
  const Corridor & corridor, const std::vector<ObstacleCluster> & clusters,
  const Trajectory * selected)
{
  (void)corridor;
  (void)clusters;
  visualization_msgs::msg::MarkerArray markers;
  if (selected) {
    visualization_msgs::msg::Marker line;
    line.header = selected->header;
    line.ns = "fixed_route_selected";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.15;
    line.color.g = 1.0F;
    line.color.a = 0.9F;
    for (const auto & point : selected->points) {
      line.points.push_back(point.pose.position);
    }
    markers.markers.push_back(std::move(line));
  }
  markers_pub_->publish(markers);
}

}  // namespace autoware::fixed_route_obstacle_bypass

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::fixed_route_obstacle_bypass::FixedRouteObstacleBypassPlannerNode)
