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
#include "test_utils.hpp"

#include <boost/geometry/algorithms/intersects.hpp>

#include <gtest/gtest.h>

namespace autoware::fixed_route_obstacle_bypass
{

TEST(FixedRouteGeometry, RotatedFootprintMustBeFullyInsideOriginalCorridor)
{
  MultiPolygon2d corridor;
  corridor.push_back(test::rectangle(-5.0, -3.5, 5.0, 3.5));
  EXPECT_TRUE(polygon_covered_by(
    vehicle_footprint(test::pose(0.0, 0.0, 0.7), test::vehicle_info()), corridor, 1.0e-6));
  EXPECT_FALSE(polygon_covered_by(
    vehicle_footprint(test::pose(0.0, 2.5, 0.7), test::vehicle_info()), corridor, 1.0e-6));
}

TEST(FixedRouteGeometry, SweptFootprintDetectsBetweenSampleTunneling)
{
  const auto sweeps =
    swept_footprints(test::pose(0.0, 0.0), test::pose(5.0, 0.0), test::vehicle_info(), 0.25, 0.035);
  const auto obstacle = test::rectangle(2.4, -0.2, 2.6, 0.2);
  EXPECT_GT(sweeps.size(), 1U);
  EXPECT_TRUE(std::any_of(sweeps.begin(), sweeps.end(), [&](const auto & sweep) {
    return boost::geometry::intersects(sweep, obstacle);
  }));
}

TEST(FixedRouteGeometry, NumericalGapClosingDoesNotGrowIntoAdjacentLane)
{
  const auto route =
    union_polygons({test::rectangle(0.0, -2.0, 5.0, 2.0), test::rectangle(5.02, -2.0, 10.0, 2.0)});
  const auto closed = buffer_polygon(buffer_polygon(route, 0.03), -0.03);
  EXPECT_FALSE(closed.empty());
  EXPECT_LT(polygon_area(closed), 41.0);
}

}  // namespace autoware::fixed_route_obstacle_bypass
