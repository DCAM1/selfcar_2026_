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

#include "autoware/fixed_route_obstacle_bypass_planner/corridor.hpp"

#include "autoware/fixed_route_obstacle_bypass_planner/geometry.hpp"

#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{
namespace bg = boost::geometry;

namespace
{
template <class T>
void hash_combine(std::size_t & seed, const T & value)
{
  seed ^= std::hash<T>{}(value) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
}

std::size_t corridor_path_signature(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  std::size_t signature = 0U;
  std::vector<int64_t> previous_lane_ids;
  bool first_run = true;
  for (const auto & point : path.points) {
    auto lane_ids = point.lane_ids;
    std::sort(lane_ids.begin(), lane_ids.end());
    lane_ids.erase(std::unique(lane_ids.begin(), lane_ids.end()), lane_ids.end());
    if (lane_ids == previous_lane_ids) {
      continue;
    }
    hash_combine(signature, lane_ids.size());
    for (const auto lane_id : lane_ids) {
      hash_combine(signature, lane_id);
    }
    // The first point of a rolling local path can move with Ego without
    // changing the route corridor. Subsequent lane-id run boundaries define
    // the actual source/target transition locations and must invalidate the
    // cached polygon when they move.
    if (!first_run) {
      // PathWithLaneId is spatially cropped/resampled as Ego advances, so a
      // lane-run boundary normally moves by a few centimetres every cycle.
      // The corridor already has a 5 m transition overlap and is rebuilt
      // after 2.5 m of projected Ego progress; metre quantization prevents
      // those harmless resampling changes from defeating the cache.
      constexpr double transition_quantization_m = 1.0;
      hash_combine(
        signature,
        static_cast<int64_t>(
          std::llround(point.point.pose.position.x / transition_quantization_m)));
      hash_combine(
        signature,
        static_cast<int64_t>(
          std::llround(point.point.pose.position.y / transition_quantization_m)));
    }
    previous_lane_ids = std::move(lane_ids);
    first_run = false;
  }
  return signature;
}

struct PathProjection
{
  size_t nearest_index{0};
  double path_s{0.0};
  std::vector<double> arc_lengths;
};

std::vector<double> path_arc_lengths(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  std::vector<double> arc(path.points.size(), 0.0);
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & previous = path.points[i - 1].point.pose.position;
    const auto & current = path.points[i].point.pose.position;
    arc[i] = arc[i - 1] + std::hypot(current.x - previous.x, current.y - previous.y);
  }
  return arc;
}

std::optional<PathProjection> project_to_local_path(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & ego_pose)
{
  if (path.points.size() < 2) {
    return std::nullopt;
  }
  const auto pose_cost = [&ego_pose](const auto & point) {
    const double dx = point.point.pose.position.x - ego_pose.position.x;
    const double dy = point.point.pose.position.y - ego_pose.position.y;
    const double yaw_difference = std::atan2(
      std::sin(tf2::getYaw(point.point.pose.orientation) - tf2::getYaw(ego_pose.orientation)),
      std::cos(tf2::getYaw(point.point.pose.orientation) - tf2::getYaw(ego_pose.orientation)));
    constexpr double yaw_cost_scale_m = 5.0;
    return dx * dx + dy * dy +
           yaw_cost_scale_m * yaw_cost_scale_m * yaw_difference * yaw_difference;
  };
  const auto nearest = std::min_element(
    path.points.begin(), path.points.end(),
    [&pose_cost](const auto & lhs, const auto & rhs) { return pose_cost(lhs) < pose_cost(rhs); });
  const size_t nearest_index = static_cast<size_t>(std::distance(path.points.begin(), nearest));
  const auto arc = path_arc_lengths(path);

  double projected_s = arc[nearest_index];
  double best_distance = std::numeric_limits<double>::infinity();
  const size_t first_segment = nearest_index > 0 ? nearest_index - 1 : 0;
  const size_t last_segment = std::min(nearest_index, path.points.size() - 2);
  for (size_t i = first_segment; i <= last_segment; ++i) {
    const auto & from = path.points[i].point.pose.position;
    const auto & to = path.points[i + 1].point.pose.position;
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double squared_length = dx * dx + dy * dy;
    if (squared_length < 1.0e-8) {
      continue;
    }
    const double ratio = std::clamp(
      ((ego_pose.position.x - from.x) * dx + (ego_pose.position.y - from.y) * dy) / squared_length,
      0.0, 1.0);
    const double x = from.x + ratio * dx;
    const double y = from.y + ratio * dy;
    const double distance = std::hypot(x - ego_pose.position.x, y - ego_pose.position.y);
    if (distance < best_distance) {
      best_distance = distance;
      projected_s = arc[i] + ratio * std::sqrt(squared_length);
    }
  }
  return PathProjection{nearest_index, projected_s, arc};
}

std::unordered_set<int64_t> route_approved_lane_ids(
  const autoware_planning_msgs::msg::LaneletRoute & route)
{
  std::unordered_set<int64_t> approved;
  for (const auto & segment : route.segments) {
    if (segment.preferred_primitive.primitive_type != "area") {
      approved.insert(segment.preferred_primitive.id);
    }
    for (const auto & primitive : segment.primitives) {
      if (primitive.primitive_type != "area") {
        approved.insert(primitive.id);
      }
    }
  }
  return approved;
}

Point2d path_point_at_s(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const std::vector<double> & arc, const double query_s)
{
  if (query_s <= 0.0) {
    const auto & first = path.points.front().point.pose.position;
    const auto & second = path.points[1].point.pose.position;
    const double length = std::hypot(second.x - first.x, second.y - first.y);
    const double scale = length > 1.0e-6 ? query_s / length : 0.0;
    return {first.x + scale * (second.x - first.x), first.y + scale * (second.y - first.y)};
  }
  if (query_s >= arc.back()) {
    const auto & previous = path.points[path.points.size() - 2].point.pose.position;
    const auto & last = path.points.back().point.pose.position;
    const double length = std::hypot(last.x - previous.x, last.y - previous.y);
    const double scale = length > 1.0e-6 ? (query_s - arc.back()) / length : 0.0;
    return {last.x + scale * (last.x - previous.x), last.y + scale * (last.y - previous.y)};
  }
  const auto upper = std::upper_bound(arc.begin(), arc.end(), query_s);
  const size_t after = static_cast<size_t>(std::distance(arc.begin(), upper));
  const size_t before = after - 1;
  const double span = arc[after] - arc[before];
  const double ratio = span > 1.0e-6 ? (query_s - arc[before]) / span : 0.0;
  const auto & from = path.points[before].point.pose.position;
  const auto & to = path.points[after].point.pose.position;
  return {from.x + ratio * (to.x - from.x), from.y + ratio * (to.y - from.y)};
}

MultiPolygon2d make_interval_mask(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const PathProjection & projection, const double start_s, const double end_s,
  const double half_width_m)
{
  const double global_start = projection.path_s + start_s;
  const double global_end = projection.path_s + end_s;
  if (global_end <= global_start + 1.0e-3 || half_width_m <= 0.0) {
    return {};
  }
  autoware_utils_geometry::LineString2d line;
  line.push_back(path_point_at_s(path, projection.arc_lengths, global_start));
  for (size_t i = 0; i < projection.arc_lengths.size(); ++i) {
    if (projection.arc_lengths[i] > global_start && projection.arc_lengths[i] < global_end) {
      const auto & point = path.points[i].point.pose.position;
      line.emplace_back(point.x, point.y);
    }
  }
  line.push_back(path_point_at_s(path, projection.arc_lengths, global_end));

  MultiPolygon2d mask;
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(half_width_m);
  bg::strategy::buffer::side_straight side_strategy;
  bg::strategy::buffer::join_round join_strategy(16);
  bg::strategy::buffer::end_flat end_strategy;
  bg::strategy::buffer::point_circle circle_strategy(16);
  bg::buffer(
    line, mask, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
  for (auto & polygon : mask) {
    bg::correct(polygon);
  }
  return mask;
}

std::optional<size_t> route_segment_for_lane(
  const autoware_planning_msgs::msg::LaneletRoute & route, const int64_t lane_id)
{
  for (size_t i = 0; i < route.segments.size(); ++i) {
    if (route.segments[i].preferred_primitive.id == lane_id) {
      return i;
    }
    const auto & primitives = route.segments[i].primitives;
    if (std::any_of(primitives.begin(), primitives.end(), [&](const auto & primitive) {
          return primitive.id == lane_id;
        })) {
      return i;
    }
  }
  return std::nullopt;
}
}  // namespace

std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto value : uuid.uuid) {
    stream << std::setw(2) << static_cast<unsigned int>(value);
  }
  return stream.str();
}

std::vector<LaneInterval> select_local_route_lane_intervals(
  const autoware_planning_msgs::msg::LaneletRoute & route,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & ego_pose, const double horizon_m,
  const double transition_overlap_m)
{
  const auto approved_ids = route_approved_lane_ids(route);
  const auto projection = project_to_local_path(path, ego_pose);
  if (approved_ids.empty() || !projection || horizon_m <= 0.0) {
    return {};
  }

  std::vector<LaneInterval> intervals;
  struct ActiveInterval
  {
    size_t interval_index{0};
    size_t last_path_index{0};
  };
  std::unordered_map<int64_t, ActiveInterval> active_intervals;
  const auto observe = [&](const int64_t lane_id, const double local_s, const size_t path_index) {
    if (approved_ids.count(lane_id) == 0U) {
      return;
    }
    const auto active = active_intervals.find(lane_id);
    if (active == active_intervals.end() || path_index > active->second.last_path_index + 1) {
      intervals.push_back(LaneInterval{lane_id, local_s, local_s});
      active_intervals[lane_id] = ActiveInterval{intervals.size() - 1, path_index};
      return;
    }
    auto & interval = intervals[active->second.interval_index];
    interval.start_s = std::min(interval.start_s, local_s);
    interval.end_s = std::max(interval.end_s, local_s);
    active->second.last_path_index = path_index;
  };

  for (const auto lane_id : path.points[projection->nearest_index].lane_ids) {
    observe(lane_id, 0.0, projection->nearest_index);
  }
  for (size_t i = projection->nearest_index; i < path.points.size(); ++i) {
    const double local_s = projection->arc_lengths[i] - projection->path_s;
    if (local_s < -1.0e-3) {
      continue;
    }
    if (local_s > horizon_m + transition_overlap_m) {
      break;
    }
    for (const auto lane_id : path.points[i].lane_ids) {
      observe(lane_id, std::clamp(local_s, 0.0, horizon_m), i);
    }
  }
  for (auto & interval : intervals) {
    interval.start_s = std::max(0.0, interval.start_s - transition_overlap_m);
    interval.end_s = std::min(horizon_m, interval.end_s + transition_overlap_m);
  }
  return intervals;
}

std::vector<int64_t> select_local_route_lane_ids(
  const autoware_planning_msgs::msg::LaneletRoute & route,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & ego_pose, const double horizon_m,
  const double transition_overlap_m)
{
  std::vector<int64_t> ids;
  for (const auto & interval :
       select_local_route_lane_intervals(route, path, ego_pose, horizon_m, transition_overlap_m)) {
    ids.push_back(interval.lane_id);
  }
  return ids;
}

void RouteCorridorBuilder::update_map(const autoware_map_msgs::msg::LaneletMapBin & map)
{
  cached_corridor_.reset();
  map_ = map;
  route_handler_.setMap(map);
  route_handler_.setAllowArea(false);
  if (route_) {
    route_handler_.setRoute(*route_);
  }
}

void RouteCorridorBuilder::update_route(
  const autoware_planning_msgs::msg::LaneletRoute & route, const rclcpp::Time & received_at)
{
  cached_corridor_.reset();
  const std::string old_uuid = route_ ? uuid_to_string(route_->uuid) : std::string{};
  const std::string new_uuid = uuid_to_string(route.uuid);
  if (!route_ || old_uuid != new_uuid) {
    ++route_generation_;
    path_.reset();
    path_route_generation_ = 0;
  }
  route_ = route;
  route_received_at_ = received_at;
  if (map_) {
    route_handler_.setRoute(route);
  }
}

void RouteCorridorBuilder::update_path(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const rclcpp::Time & received_at)
{
  const auto new_signature = corridor_path_signature(path);
  if (!path_ || new_signature != path_signature_) {
    cached_corridor_.reset();
  }
  path_signature_ = new_signature;
  path_ = path;
  path_received_at_ = received_at;
  path_route_generation_ = route_generation_;
}

bool RouteCorridorBuilder::ready() const
{
  return map_.has_value() && route_.has_value() && path_.has_value() &&
         path_route_generation_ == route_generation_ && route_handler_.isHandlerReady();
}

uint64_t RouteCorridorBuilder::route_generation() const
{
  return route_generation_;
}

std::string RouteCorridorBuilder::route_uuid() const
{
  return route_ ? uuid_to_string(route_->uuid) : std::string{};
}

std::optional<Corridor> RouteCorridorBuilder::build(
  const geometry_msgs::msg::Pose & ego_pose, const double horizon_m,
  const double vehicle_half_width_m, const Parameters & parameters, std::string * error,
  CandidateFailureReason * failure_reason) const
{
  const auto fail = [&](
                      const CandidateFailureReason reason,
                      const std::string & message) -> std::optional<Corridor> {
    if (error) {
      *error = message;
    }
    if (failure_reason) {
      *failure_reason = reason;
    }
    return std::nullopt;
  };
  if (!ready()) {
    return fail(
      CandidateFailureReason::CORRIDOR_DISCONNECTED, "route corridor inputs are not synchronized");
  }

  const auto projection = project_to_local_path(*path_, ego_pose);
  if (!projection) {
    return fail(
      CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY,
      "failed to project ego onto PathWithLaneId");
  }

  const Point2d ego_center{ego_pose.position.x, ego_pose.position.y};
  // Corridor construction performs lanelet resolution, clipping, polygon
  // union and buffering. Those polygons are spatially invariant while the
  // active route and the PathWithLaneId transition sequence are unchanged.
  // Build a short extra forward section and reuse it only while that section
  // still fully covers the requested Ego-relative horizon. Every output is
  // still checked with its rotated and swept footprint against this polygon.
  const double cache_extra_horizon_m =
    std::max(1.0, parameters.corridor_endpoint_buffer_m);
  const double cache_rebuild_advance_m = 0.5 * cache_extra_horizon_m;
  if (cached_corridor_) {
    const double forward_advance_m = projection->path_s - cached_corridor_->path_s;
    const bool cache_key_matches =
      cached_corridor_->route_generation == route_generation_ &&
      cached_corridor_->path_signature == path_signature_ &&
      std::abs(cached_corridor_->vehicle_half_width_m - vehicle_half_width_m) < 1.0e-6;
    const bool horizon_is_covered =
      horizon_m + std::max(0.0, forward_advance_m) + 0.25 <=
      cached_corridor_->built_horizon_m;
    const bool ego_is_covered = bg::covered_by(ego_center, cached_corridor_->corridor.original);
    if (
      cache_key_matches && forward_advance_m >= -cache_rebuild_advance_m &&
      forward_advance_m <= cache_rebuild_advance_m && horizon_is_covered && ego_is_covered) {
      return cached_corridor_->corridor;
    }
  }

  const double build_horizon_m = horizon_m + cache_extra_horizon_m;
  auto lane_intervals = select_local_route_lane_intervals(
    *route_, *path_, ego_pose, build_horizon_m, parameters.corridor_transition_overlap_m);
  if (lane_intervals.empty()) {
    return fail(
      CandidateFailureReason::EGO_LANE_NOT_IN_CORRIDOR,
      "no route-approved local "
      "PathWithLaneId lane");
  }

  std::unordered_map<int64_t, Polygon2d> lane_polygons;
  const auto resolve_lane_polygon = [&](const int64_t lane_id) -> std::optional<Polygon2d> {
    try {
      const auto lanelet = route_handler_.getLaneletsFromId(lane_id);
      Polygon2d polygon;
      for (const auto & point : lanelet.polygon2d().basicPolygon()) {
        polygon.outer().emplace_back(point.x(), point.y());
      }
      boost::geometry::correct(polygon);
      return polygon;
    } catch (const std::exception &) {
      return std::nullopt;
    }
  };
  try {
    for (const auto & interval : lane_intervals) {
      const auto polygon = resolve_lane_polygon(interval.lane_id);
      if (!polygon) {
        return fail(
          CandidateFailureReason::CORRIDOR_DISCONNECTED,
          "failed to resolve route lanelet " + std::to_string(interval.lane_id));
      }
      lane_polygons.emplace(interval.lane_id, *polygon);
    }
  } catch (const std::exception & exception) {
    return fail(
      CandidateFailureReason::CORRIDOR_DISCONNECTED,
      std::string("failed to resolve route lanelet: ") + exception.what());
  }

  int64_t ego_lane_id = 0;
  const auto choose_ego_lane = [&](const std::vector<int64_t> & preferred_order) {
    for (const auto lane_id : preferred_order) {
      const auto polygon = lane_polygons.find(lane_id);
      if (polygon != lane_polygons.end() && bg::covered_by(ego_center, polygon->second)) {
        return lane_id;
      }
    }
    for (const auto & interval : lane_intervals) {
      const auto polygon = lane_polygons.find(interval.lane_id);
      if (polygon != lane_polygons.end() && bg::covered_by(ego_center, polygon->second)) {
        return interval.lane_id;
      }
    }
    return int64_t{0};
  };
  const auto & ego_path_lane_ids = path_->points[projection->nearest_index].lane_ids;
  ego_lane_id = choose_ego_lane(ego_path_lane_ids);

  if (ego_lane_id == 0) {
    const auto active_segment = route_segment_for_lane(*route_, lane_intervals.front().lane_id);
    if (!active_segment) {
      return fail(
        CandidateFailureReason::EGO_LANE_NOT_IN_CORRIDOR, "active route segment was not found");
    }
    std::vector<int64_t> fallback_ids;
    const auto append_segment_primitives = [&](const size_t segment_index) {
      const auto & segment = route_->segments[segment_index];
      for (const auto & primitive : segment.primitives) {
        if (
          primitive.primitive_type != "area" &&
          std::find(fallback_ids.begin(), fallback_ids.end(), primitive.id) == fallback_ids.end()) {
          fallback_ids.push_back(primitive.id);
        }
      }
      if (
        segment.preferred_primitive.primitive_type != "area" &&
        std::find(fallback_ids.begin(), fallback_ids.end(), segment.preferred_primitive.id) ==
          fallback_ids.end()) {
        fallback_ids.push_back(segment.preferred_primitive.id);
      }
    };
    // At a route-segment transition the first forward PathWithLaneId can
    // already name the target segment while Ego is still in the source
    // segment. Search only that target and its immediate predecessor, never
    // arbitrary nearby map lanelets.
    if (*active_segment > 0) {
      append_segment_primitives(*active_segment - 1);
    }
    append_segment_primitives(*active_segment);
    for (const auto lane_id : fallback_ids) {
      if (lane_polygons.count(lane_id) == 0U) {
        const auto polygon = resolve_lane_polygon(lane_id);
        if (polygon) {
          lane_polygons.emplace(lane_id, *polygon);
        }
      }
    }
    ego_lane_id = choose_ego_lane(fallback_ids);
    if (ego_lane_id == 0) {
      return fail(
        CandidateFailureReason::EGO_LANE_NOT_IN_CORRIDOR,
        "ego is outside active "
        "RouteSegment primitives");
    }
    if (std::none_of(lane_intervals.begin(), lane_intervals.end(), [&](const auto & interval) {
          return interval.lane_id == ego_lane_id;
        })) {
      lane_intervals.insert(
        lane_intervals.begin(),
        LaneInterval{
          ego_lane_id, 0.0, std::min(build_horizon_m, parameters.corridor_transition_overlap_m)});
    }
  }

  const auto ego_interval =
    std::find_if(lane_intervals.begin(), lane_intervals.end(), [&](const auto & interval) {
      return interval.lane_id == ego_lane_id && interval.start_s <= 1.0e-3;
    });
  if (ego_interval != lane_intervals.end() && ego_interval != lane_intervals.begin()) {
    std::rotate(lane_intervals.begin(), ego_interval, std::next(ego_interval));
  }

  std::vector<Polygon2d> clipped_lane_polygons;
  for (const auto & interval : lane_intervals) {
    const double mask_start = interval.start_s <= 1.0e-3
                                ? interval.start_s - parameters.corridor_endpoint_buffer_m
                                : interval.start_s;
    const double mask_end = interval.end_s >= build_horizon_m - 1.0e-3
                              ? interval.end_s + parameters.corridor_endpoint_buffer_m
                              : interval.end_s;
    const auto mask = make_interval_mask(
      *path_, *projection, mask_start, mask_end, parameters.corridor_path_clip_half_width_m);
    MultiPolygon2d clipped;
    bg::intersection(lane_polygons.at(interval.lane_id), mask, clipped);
    for (auto & polygon : clipped) {
      bg::correct(polygon);
      clipped_lane_polygons.push_back(std::move(polygon));
    }
  }
  if (clipped_lane_polygons.empty()) {
    return fail(
      CandidateFailureReason::CORRIDOR_DISCONNECTED,
      "local lane interval clipping produced "
      "no polygon");
  }

  auto original = union_polygons(clipped_lane_polygons);
  if (original.empty()) {
    return fail(
      CandidateFailureReason::CORRIDOR_DISCONNECTED, "local route lanelet union is empty");
  }

  // Close only map-scale numerical cracks. The subsequent inward boundary
  // margin is larger than this tolerance, so this operation cannot be used to
  // enter an adjacent non-route lanelet.
  if (parameters.route_gap_tolerance_m > 0.0) {
    const auto expanded = buffer_polygon(original, parameters.route_gap_tolerance_m);
    const auto closed = buffer_polygon(expanded, -parameters.route_gap_tolerance_m);
    if (!closed.empty()) {
      original = closed;
    }
  }

  const auto ego_component = std::find_if(
    original.begin(), original.end(),
    [&](const auto & polygon) { return bg::covered_by(ego_center, polygon); });
  if (ego_component == original.end()) {
    return fail(
      CandidateFailureReason::EGO_LANE_NOT_IN_CORRIDOR, "local corridor does not contain ego");
  }
  MultiPolygon2d connected_original;
  connected_original.push_back(*ego_component);
  original = std::move(connected_original);

  const double center_inset = vehicle_half_width_m + parameters.route_boundary_margin_m;
  auto center_space = buffer_polygon(original, -center_inset);
  if (center_space.empty()) {
    return fail(
      CandidateFailureReason::CORRIDOR_DISCONNECTED,
      "route corridor is narrower than the vehicle and boundary margin");
  }

  Corridor corridor;
  corridor.original = std::move(original);
  corridor.center_space = std::move(center_space);
  for (const auto & interval : lane_intervals) {
    corridor.ordered_lane_ids.push_back(interval.lane_id);
  }
  corridor.lane_intervals = std::move(lane_intervals);
  corridor.ego_path_lane_ids.assign(ego_path_lane_ids.begin(), ego_path_lane_ids.end());
  corridor.ego_lane_id = ego_lane_id;
  corridor.route_uuid = uuid_to_string(route_->uuid);
  corridor.route_generation = route_generation_;
  cached_corridor_ = CachedCorridor{
    corridor, projection->path_s, build_horizon_m, vehicle_half_width_m, route_generation_,
    path_signature_};
  if (failure_reason) {
    *failure_reason = CandidateFailureReason::NONE;
  }
  return corridor;
}

bool RouteCorridorBuilder::input_is_current(
  const builtin_interfaces::msg::Time & trajectory_stamp, const double max_skew_s,
  std::string * error, const bool require_trajectory_path_stamp_alignment) const
{
  const auto fail = [&](const std::string & message) {
    if (error) {
      *error = message;
    }
    return false;
  };
  if (!ready()) {
    return fail("route, map, or post-route PathWithLaneId is missing");
  }
  if (path_received_at_ < route_received_at_) {
    return fail("PathWithLaneId predates the active route update");
  }

  const rclcpp::Time trajectory_time(trajectory_stamp, RCL_ROS_TIME);
  const rclcpp::Time path_time(path_->header.stamp, RCL_ROS_TIME);
  const rclcpp::Time route_time(route_->header.stamp, RCL_ROS_TIME);
  if (
    require_trajectory_path_stamp_alignment && trajectory_time.nanoseconds() > 0 &&
    path_time.nanoseconds() > 0) {
    if (std::abs((trajectory_time - path_time).seconds()) > max_skew_s) {
      return fail("optimized trajectory and PathWithLaneId timestamps are not aligned");
    }
  }
  if (path_time.nanoseconds() > 0 && route_time.nanoseconds() > 0) {
    if ((path_time - route_time).seconds() < -max_skew_s) {
      return fail("PathWithLaneId timestamp predates the active route");
    }
  }
  return true;
}

}  // namespace autoware::fixed_route_obstacle_bypass
