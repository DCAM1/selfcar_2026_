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
#include "test_utils.hpp"

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/centroid.hpp>

#include <gtest/gtest.h>

namespace autoware::fixed_route_obstacle_bypass
{
namespace
{
void update_track(
  ObjectTracker & tracker, autoware_perception_msgs::msg::PredictedObject object,
  const double stamp_s)
{
  autoware_perception_msgs::msg::PredictedObjects objects;
  objects.header.stamp = test::ros_time(stamp_s);
  objects.objects.push_back(std::move(object));
  tracker.update(objects, test::ros_time(stamp_s));
}

autoware::sampler_common::transform::Spline2D straight_reference()
{
  return {{0.0, 10.0, 20.0}, {0.0, 0.0, 0.0}};
}
}  // namespace

TEST(GenericObjectTracker, RemainsClassFreeAndBuildsOccupancyFromShape)
{
  ObjectTracker tracker{Parameters{}};
  update_track(tracker, test::object(1, 5.0, 0.0, 0.0, 0.0), 1.0);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  const auto & track = tracker.tracks().begin()->second;
  const auto occupancies = tracker.occupancies_at(track, test::ros_time(1.0), 0.0);
  ASSERT_FALSE(occupancies.empty());
  EXPECT_GT(std::abs(boost::geometry::area(occupancies.front().polygon)), 1.0);
}

TEST(GenericObjectTracker, ClassifiesRouteRelativeMotionWithoutSemanticLabels)
{
  Parameters parameters;
  parameters.history_min_samples = 3;
  ObjectTracker tracker{parameters};
  for (int i = 0; i < 3; ++i) {
    update_track(tracker, test::object(1, 5.0, 0.0, 0.0, 0.0), 1.0 + 0.1 * i);
  }
  EXPECT_EQ(
    tracker.classify_motion(
      tracker.tracks().begin()->second, straight_reference(), test::ros_time(1.2)),
    MotionState::STATIONARY);

  ObjectTracker crossing{parameters};
  for (int i = 0; i < 3; ++i) {
    update_track(crossing, test::object(2, 5.0, -1.0 + 0.1 * i, M_PI_2, 1.0), 2.0 + 0.1 * i);
  }
  EXPECT_EQ(
    crossing.classify_motion(
      crossing.tracks().begin()->second, straight_reference(), test::ros_time(2.2)),
    MotionState::CROSSING);
}

TEST(GenericObjectTracker, UsesAssociationFallbackWhenUuidChanges)
{
  ObjectTracker tracker{Parameters{}};
  update_track(tracker, test::object(1, 5.0, 0.0, 0.0, 1.0), 1.0);
  update_track(tracker, test::object(2, 5.1, 0.0, 0.0, 1.0), 1.1);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_TRUE(tracker.tracks().begin()->second.uuid_reassociated);
}

TEST(GenericObjectTracker, HoldsShortDropoutAndExpiresAtTtl)
{
  Parameters parameters;
  parameters.object_dropout_ttl_s = 0.5;
  ObjectTracker tracker{parameters};
  update_track(tracker, test::object(1, 5.0, 0.0, 0.0, 0.0), 1.0);
  tracker.prune(test::ros_time(1.4));
  EXPECT_EQ(tracker.tracks().size(), 1U);
  tracker.prune(test::ros_time(1.6));
  EXPECT_TRUE(tracker.tracks().empty());
}

TEST(GenericObjectTracker, CompensatesMessageAgeInMapFrame)
{
  ObjectTracker tracker{Parameters{}};
  update_track(tracker, test::object(1, 1.0, 0.0, 0.0, 2.0), 1.0);
  const auto & track = tracker.tracks().begin()->second;
  const auto occupancy = tracker.occupancies_at(track, test::ros_time(1.5), 0.0).back().polygon;
  Point2d centroid;
  boost::geometry::centroid(occupancy, centroid);
  const auto centroid_x = centroid.x();
  EXPECT_NEAR(centroid_x, 2.0, 0.2);
}

}  // namespace autoware::fixed_route_obstacle_bypass
