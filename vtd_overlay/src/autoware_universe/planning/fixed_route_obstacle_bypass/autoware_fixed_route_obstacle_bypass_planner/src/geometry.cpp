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

#include "autoware/fixed_route_obstacle_bypass_planner/geometry.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
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

template <class Geometry>
MultiPolygon2d buffer_impl(const Geometry & geometry, const double distance)
{
  MultiPolygon2d output;
  bg::strategy::buffer::distance_symmetric<double> distance_strategy(distance);
  bg::strategy::buffer::side_straight side_strategy;
  bg::strategy::buffer::join_round join_strategy(16);
  bg::strategy::buffer::end_round end_strategy(16);
  bg::strategy::buffer::point_circle circle_strategy(16);
  bg::buffer(
    geometry, output, distance_strategy, side_strategy, join_strategy, end_strategy,
    circle_strategy);
  for (auto & polygon : output) {
    bg::correct(polygon);
  }
  return output;
}
}  // namespace

Polygon2d vehicle_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double margin)
{
  Polygon2d polygon;
  polygon.outer() = vehicle_info.createFootprint(margin, pose);
  bg::correct(polygon);
  return polygon;
}

geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to, const double ratio)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  geometry_msgs::msg::Pose pose;
  pose.position.x = from.position.x + clamped_ratio * (to.position.x - from.position.x);
  pose.position.y = from.position.y + clamped_ratio * (to.position.y - from.position.y);
  pose.position.z = from.position.z + clamped_ratio * (to.position.z - from.position.z);

  const double from_yaw = tf2::getYaw(from.orientation);
  const double yaw_delta = normalize_angle(tf2::getYaw(to.orientation) - from_yaw);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, from_yaw + clamped_ratio * yaw_delta);
  pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

Polygon2d convex_sweep(const std::vector<Polygon2d> & polygons)
{
  autoware_utils_geometry::MultiPoint2d points;
  for (const auto & polygon : polygons) {
    points.insert(points.end(), polygon.outer().begin(), polygon.outer().end());
  }
  Polygon2d hull;
  if (!points.empty()) {
    bg::convex_hull(points, hull);
    bg::correct(hull);
  }
  return hull;
}

std::vector<Polygon2d> swept_footprints(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double max_step_m,
  const double max_yaw_step_rad, const double margin, const size_t minimum_steps)
{
  const double distance =
    std::hypot(to.position.x - from.position.x, to.position.y - from.position.y);
  const double yaw_delta =
    std::abs(normalize_angle(tf2::getYaw(to.orientation) - tf2::getYaw(from.orientation)));
  const size_t distance_steps = static_cast<size_t>(
    std::ceil(distance / std::max(max_step_m, std::numeric_limits<double>::epsilon())));
  const size_t yaw_steps = static_cast<size_t>(
    std::ceil(yaw_delta / std::max(max_yaw_step_rad, std::numeric_limits<double>::epsilon())));
  const size_t steps = std::max<size_t>({1, distance_steps, yaw_steps, minimum_steps});

  std::vector<Polygon2d> result;
  result.reserve(steps);
  for (size_t i = 0; i < steps; ++i) {
    const double begin = static_cast<double>(i) / static_cast<double>(steps);
    const double end = static_cast<double>(i + 1) / static_cast<double>(steps);
    const double middle = 0.5 * (begin + end);
    result.push_back(convex_sweep(
      {vehicle_footprint(interpolate_pose(from, to, begin), vehicle_info, margin),
       vehicle_footprint(interpolate_pose(from, to, middle), vehicle_info, margin),
       vehicle_footprint(interpolate_pose(from, to, end), vehicle_info, margin)}));
  }
  return result;
}

MultiPolygon2d union_polygons(const std::vector<Polygon2d> & polygons)
{
  MultiPolygon2d result;
  for (auto polygon : polygons) {
    bg::correct(polygon);
    if (std::abs(bg::area(polygon)) < std::numeric_limits<double>::epsilon()) {
      continue;
    }
    if (result.empty()) {
      result.push_back(std::move(polygon));
      continue;
    }
    MultiPolygon2d merged;
    bg::union_(result, polygon, merged);
    if (merged.empty()) {
      result.push_back(std::move(polygon));
    } else {
      result = std::move(merged);
    }
  }
  return result;
}

MultiPolygon2d buffer_polygon(const MultiPolygon2d & polygon, const double distance)
{
  return buffer_impl(polygon, distance);
}

Polygon2d buffer_polygon(const Polygon2d & polygon, const double distance)
{
  const auto buffered = buffer_impl(polygon, distance);
  if (buffered.empty()) {
    return {};
  }
  return *std::max_element(
    buffered.begin(), buffered.end(), [](const auto & lhs, const auto & rhs) {
      return std::abs(bg::area(lhs)) < std::abs(bg::area(rhs));
    });
}

bool polygon_covered_by(
  const Polygon2d & polygon, const MultiPolygon2d & container, const double area_tolerance_m2)
{
  if (polygon.outer().empty() || container.empty()) {
    return false;
  }

  // The normal case is a footprint wholly contained by the single connected
  // corridor polygon.  Computing a polygon/multipolygon intersection for
  // every trajectory point and every swept sub-step is orders of magnitude
  // more expensive than the containment predicate.  Keep the intersection
  // below as the tolerance-aware fallback for footprints that straddle
  // multiple components or differ only by map-scale numerical error.
  for (const auto & component : container) {
    if (bg::covered_by(polygon, component)) {
      return true;
    }
  }

  MultiPolygon2d intersections;
  bg::intersection(polygon, container, intersections);
  const double uncovered_area = std::abs(bg::area(polygon)) - std::abs(polygon_area(intersections));
  return uncovered_area <= std::max(0.0, area_tolerance_m2);
}

double polygon_area(const MultiPolygon2d & polygons)
{
  double area = 0.0;
  for (const auto & polygon : polygons) {
    area += std::abs(bg::area(polygon));
  }
  return area;
}

}  // namespace autoware::fixed_route_obstacle_bypass
