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

#ifndef AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__GEOMETRY_HPP_
#define AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__GEOMETRY_HPP_

#include "autoware/fixed_route_obstacle_bypass_planner/types.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{

Polygon2d vehicle_footprint(
  const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, double margin = 0.0);

geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to, double ratio);

Polygon2d convex_sweep(const std::vector<Polygon2d> & polygons);

std::vector<Polygon2d> swept_footprints(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, double max_step_m,
  double max_yaw_step_rad, double margin = 0.0, size_t minimum_steps = 1);

MultiPolygon2d union_polygons(const std::vector<Polygon2d> & polygons);
MultiPolygon2d buffer_polygon(const MultiPolygon2d & polygon, double distance);
Polygon2d buffer_polygon(const Polygon2d & polygon, double distance);

bool polygon_covered_by(
  const Polygon2d & polygon, const MultiPolygon2d & container, double area_tolerance_m2);

double polygon_area(const MultiPolygon2d & polygons);

}  // namespace autoware::fixed_route_obstacle_bypass

#endif  // AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__GEOMETRY_HPP_
