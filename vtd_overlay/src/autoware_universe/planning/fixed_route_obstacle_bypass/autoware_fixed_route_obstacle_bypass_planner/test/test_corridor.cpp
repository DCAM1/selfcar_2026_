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
#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{
namespace
{
autoware_planning_msgs::msg::LaneletRoute make_route(const std::vector<int64_t> & ids)
{
  autoware_planning_msgs::msg::LaneletRoute route;
  for (const auto id : ids) {
    autoware_planning_msgs::msg::LaneletSegment segment;
    segment.preferred_primitive.id = id;
    segment.preferred_primitive.primitive_type = "lane";
    route.segments.push_back(segment);
  }
  return route;
}

autoware_planning_msgs::msg::LaneletRoute make_route_segment(
  const int64_t preferred_id, const std::vector<int64_t> & primitive_ids)
{
  autoware_planning_msgs::msg::LaneletRoute route;
  autoware_planning_msgs::msg::LaneletSegment segment;
  segment.preferred_primitive.id = preferred_id;
  segment.preferred_primitive.primitive_type = "lane";
  for (const auto id : primitive_ids) {
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = id;
    primitive.primitive_type = "lane";
    segment.primitives.push_back(primitive);
  }
  route.segments.push_back(segment);
  return route;
}

void add_path_point(
  autoware_internal_planning_msgs::msg::PathWithLaneId & path, const double x,
  const std::vector<int64_t> & lane_ids, const double yaw = 0.0)
{
  autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose = test::pose(x, 0.0, yaw);
  point.lane_ids = lane_ids;
  path.points.push_back(point);
}
}  // namespace

TEST(RouteCorridorSelection, IncludesOnlyOrderedRouteApprovedLanelets)
{
  const auto route = make_route({10, 20});
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  add_path_point(path, 0.0, {10, 999});
  add_path_point(path, 5.0, {10, 20, 999});
  add_path_point(path, 10.0, {20});

  EXPECT_EQ(
    select_local_route_lane_ids(route, path, test::pose(0.0, 0.0), 20.0),
    (std::vector<int64_t>{10, 20}));
}

TEST(RouteCorridorSelection, IncludesRouteDirectedSourceAndTargetAtTransition)
{
  const auto route = make_route({101, 202});
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  add_path_point(path, 0.0, {101});
  add_path_point(path, 2.0, {101, 202});
  add_path_point(path, 4.0, {202});

  EXPECT_EQ(
    select_local_route_lane_ids(route, path, test::pose(1.0, 0.0), 10.0),
    (std::vector<int64_t>{101, 202}));
}

TEST(RouteCorridorSelection, RejectsAdjacentAndOppositeNonRouteLanelets)
{
  const auto route = make_route({10});
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  add_path_point(path, 0.0, {90, 91});
  add_path_point(path, 5.0, {90});
  EXPECT_TRUE(select_local_route_lane_ids(route, path, test::pose(0.0, 0.0), 20.0).empty());
}

TEST(RouteCorridorSelection, CropsAtLocalPlanningHorizon)
{
  const auto route = make_route({10, 20});
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  add_path_point(path, 0.0, {10});
  add_path_point(path, 5.0, {10});
  add_path_point(path, 50.0, {20});
  EXPECT_EQ(
    select_local_route_lane_ids(route, path, test::pose(0.0, 0.0), 10.0),
    (std::vector<int64_t>{10}));
}

TEST(RouteCorridorSelection, DoesNotJumpToNearbyOppositeRouteSection)
{
  const auto route = make_route({10, 20});
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  add_path_point(path, 1.0, {10}, 0.0);
  add_path_point(path, 5.0, {10}, 0.0);
  add_path_point(path, 0.1, {20}, M_PI);

  EXPECT_EQ(
    select_local_route_lane_ids(route, path, test::pose(0.0, 0.0, 0.0), 5.0),
    (std::vector<int64_t>{10}));
}

TEST(RouteCorridorSelection, KeepsRepeatedLaneIdAsSeparateOrderedIntervals)
{
  const auto route = make_route({10, 20});
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  add_path_point(path, 0.0, {10});
  add_path_point(path, 5.0, {10});
  add_path_point(path, 10.0, {20});
  add_path_point(path, 15.0, {20});
  add_path_point(path, 20.0, {10});
  add_path_point(path, 25.0, {10});

  const auto intervals =
    select_local_route_lane_intervals(route, path, test::pose(0.0, 0.0), 30.0, 0.0);
  ASSERT_EQ(intervals.size(), 3U);
  EXPECT_EQ(intervals.at(0).lane_id, 10);
  EXPECT_EQ(intervals.at(1).lane_id, 20);
  EXPECT_EQ(intervals.at(2).lane_id, 10);
  EXPECT_LT(intervals.at(0).end_s, intervals.at(2).start_s);
}

TEST(RouteCorridorSelection, UsesCurrentSourceThenPreferredTargetAndExcludesUnusedAlternative)
{
  const auto route = make_route_segment(41922, {41922, 42181, 42440});
  autoware_internal_planning_msgs::msg::PathWithLaneId path;
  add_path_point(path, 236.220, {42181});
  add_path_point(path, 241.220, {42181});
  add_path_point(path, 251.220, {42181, 41922});
  add_path_point(path, 261.220, {41922});
  add_path_point(path, 281.220, {41922});
  for (auto & point : path.points) {
    point.point.pose.position.y = 141.733;
  }

  const auto intervals =
    select_local_route_lane_intervals(route, path, test::pose(241.220, 141.733), 35.0, 5.0);
  ASSERT_EQ(intervals.size(), 2U);
  EXPECT_EQ(intervals.at(0).lane_id, 42181);
  EXPECT_EQ(intervals.at(1).lane_id, 41922);
  EXPECT_EQ(
    std::count_if(
      intervals.begin(), intervals.end(),
      [](const auto & interval) { return interval.lane_id == 42440; }),
    0);
  EXPECT_LE(intervals.at(0).start_s, 0.0);
  EXPECT_GT(intervals.at(0).end_s, intervals.at(1).start_s);
}

}  // namespace autoware::fixed_route_obstacle_bypass
