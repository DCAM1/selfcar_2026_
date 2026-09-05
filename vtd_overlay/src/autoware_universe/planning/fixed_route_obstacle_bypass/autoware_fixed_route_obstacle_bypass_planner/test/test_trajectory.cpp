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
#include "test_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <gtest/gtest.h>

#include <algorithm>

namespace autoware::fixed_route_obstacle_bypass
{

TEST(FinalTrajectoryValidation, AcceptsCollisionFreePositiveProgress)
{
  auto trajectory = test::straight_trajectory();
  assign_time_from_start(trajectory.points, 2.0, 1.0);
  ObjectTracker tracker{Parameters{}};
  const auto result = validate_trajectory(
    trajectory, test::straight_corridor(), tracker, test::ros_time(1.0), test::vehicle_info(),
    Parameters{}, false);
  EXPECT_TRUE(result.valid) << result.reason;
  EXPECT_GT(progress_at_time(trajectory, 2.0), 1.0);
}

TEST(FinalTrajectoryValidation, RejectsRotatedFootprintOutsideRoute)
{
  auto trajectory = test::straight_trajectory(2.0, 3.0);
  assign_time_from_start(trajectory.points, 2.0, 1.0);
  ObjectTracker tracker{Parameters{}};
  const auto result = validate_trajectory(
    trajectory, test::straight_corridor(), tracker, test::ros_time(1.0), test::vehicle_info(),
    Parameters{}, false);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.corridor_valid);
}

TEST(FinalTrajectoryValidation, ReportsDisconnectedCorridorSeparatelyFromEmptyTrajectory)
{
  auto trajectory = test::straight_trajectory();
  assign_time_from_start(trajectory.points, 2.0, 1.0);
  ObjectTracker tracker{Parameters{}};
  const auto result = validate_trajectory(
    trajectory, Corridor{}, tracker, test::ros_time(1.0), test::vehicle_info(), Parameters{},
    false);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, CandidateFailureReason::CORRIDOR_DISCONNECTED);
}

TEST(FinalTrajectoryValidation, RejectsSweptCollisionWithStationaryUnknown)
{
  Parameters parameters;
  parameters.history_min_samples = 1;
  parameters.object_uncertainty_base_m = 0.0;
  parameters.object_uncertainty_growth_mps = 0.0;
  ObjectTracker tracker{parameters};
  autoware_perception_msgs::msg::PredictedObjects objects;
  objects.header.stamp = test::ros_time(1.0);
  objects.objects.push_back(test::object(1, 5.0, 0.0, 0.0, 0.0));
  tracker.update(objects, test::ros_time(1.0));
  auto trajectory = test::straight_trajectory();
  assign_time_from_start(trajectory.points, 2.0, 1.0);
  const auto result = validate_trajectory(
    trajectory, test::straight_corridor(), tracker, test::ros_time(1.0), test::vehicle_info(),
    parameters, false);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.collision_free);
}

TEST(FinalTrajectoryValidation, AppliesObjectUncertaintyAsSweptClearanceMargin)
{
  Parameters parameters;
  parameters.history_min_samples = 1;
  parameters.object_collision_margin_m = 0.3;
  parameters.object_uncertainty_base_m = 0.2;
  parameters.object_uncertainty_growth_mps = 0.0;
  ObjectTracker tracker{parameters};
  autoware_perception_msgs::msg::PredictedObjects objects;
  objects.header.stamp = test::ros_time(1.0);
  // The raw object and ego footprints have roughly 0.4 m lateral
  // clearance.  Their polygons do not intersect, but the configured 0.5 m
  // uncertainty/collision margin must still reject the trajectory.
  objects.objects.push_back(test::object(1, 5.0, 1.8, 0.0, 0.0));
  tracker.update(objects, test::ros_time(1.0));
  auto trajectory = test::straight_trajectory();
  assign_time_from_start(trajectory.points, 2.0, 1.0);

  const auto result = validate_trajectory(
    trajectory, test::straight_corridor(), tracker, test::ros_time(1.0), test::vehicle_info(),
    parameters, false);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, CandidateFailureReason::COLLISION_FAILURE);
}

TEST(FinalTrajectoryValidation, RechecksCollisionAfterFinalSmoothingDelaysArrival)
{
  Parameters parameters;
  parameters.history_min_samples = 1;
  parameters.object_collision_margin_m = 0.0;
  parameters.object_uncertainty_base_m = 0.0;
  parameters.object_uncertainty_growth_mps = 0.0;
  ObjectTracker tracker{parameters};
  autoware_perception_msgs::msg::PredictedObjects objects;
  objects.header.stamp = test::ros_time(1.0);
  objects.objects.push_back(test::object(1, 8.0, -7.0, M_PI_2, 2.0));
  tracker.update(objects, test::ros_time(1.0));

  auto pre_smoothing_timing = test::straight_trajectory(4.0);
  assign_time_from_start(pre_smoothing_timing.points, 4.0, 1.0);
  const auto initially_safe = validate_trajectory(
    pre_smoothing_timing, test::straight_corridor(), tracker, test::ros_time(1.0),
    test::vehicle_info(), parameters, false);
  EXPECT_TRUE(initially_safe.valid) << initially_safe.reason;

  auto delayed_final_timing = test::straight_trajectory(2.0);
  assign_time_from_start(delayed_final_timing.points, 2.0, 1.0);
  const auto collision_after_smoothing = validate_trajectory(
    delayed_final_timing, test::straight_corridor(), tracker, test::ros_time(1.0),
    test::vehicle_info(), parameters, false);
  EXPECT_FALSE(collision_after_smoothing.collision_free);
}

TEST(ZeroVelocityInvariant, RejectsObjectStyleMidPathZero)
{
  Parameters parameters;
  auto reference = test::straight_trajectory();
  for (auto & point : reference.points) {
    point.longitudinal_velocity_mps = 2.0F;
  }
  reference.points.back().longitudinal_velocity_mps = 0.0F;
  auto output = reference;
  output.points.at(8).longitudinal_velocity_mps = 0.0F;
  assign_time_from_start(output.points, 2.0, 1.0);
  const auto result = check_unexpected_zero_velocity(output, reference, 2.0, parameters);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, CandidateFailureReason::UNEXPECTED_INTERIOR_ZERO);
}

TEST(ZeroVelocityInvariant, PreservesRegulatoryZero)
{
  Parameters parameters;
  auto reference = test::straight_trajectory();
  reference.points.at(8).longitudinal_velocity_mps = 0.0F;
  auto output = reference;
  assign_time_from_start(output.points, 2.0, 1.0);
  const auto result = check_unexpected_zero_velocity(output, reference, 2.0, parameters);
  EXPECT_TRUE(result.valid);
}

TEST(ZeroVelocityInvariant, IgnoresRegulatoryStopsBehindEgoForProgressPolicy)
{
  Parameters parameters;
  auto reference = test::straight_trajectory();
  reference.points.at(2).longitudinal_velocity_mps = 0.0F;

  EXPECT_FALSE(has_nonterminal_regulatory_stop(reference, test::pose(5.0, 0.0), parameters));
  reference.points.at(15).longitudinal_velocity_mps = 0.0F;
  EXPECT_TRUE(has_nonterminal_regulatory_stop(reference, test::pose(5.0, 0.0), parameters));
}

TEST(ZeroVelocityInvariant, AllowsBoundedJerkLimitedStartupPrefix)
{
  Parameters parameters;
  auto reference = test::straight_trajectory();
  auto output = reference;
  output.points.at(0).longitudinal_velocity_mps = 0.0F;
  output.points.at(1).longitudinal_velocity_mps = 0.000282F;
  output.points.at(2).longitudinal_velocity_mps = 0.280032F;
  assign_time_from_start(output.points, 0.0, 1.0);

  const auto result = check_unexpected_zero_velocity(output, reference, 0.0, parameters);
  EXPECT_TRUE(result.valid);
  ASSERT_TRUE(result.startup_release_index.has_value());
  EXPECT_EQ(*result.startup_release_index, 2U);
}

TEST(ZeroVelocityInvariant, DoesNotReleaseOnSingleVelocitySpike)
{
  Parameters parameters;
  auto reference = test::straight_trajectory();
  auto output = reference;
  for (auto & point : output.points) {
    point.longitudinal_velocity_mps = 0.01F;
  }
  output.points.at(2).longitudinal_velocity_mps = 0.30F;
  output.points.back().longitudinal_velocity_mps = 0.0F;
  assign_time_from_start(output.points, 0.0, 1.0);

  const auto result = check_unexpected_zero_velocity(output, reference, 0.0, parameters);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, CandidateFailureReason::ALL_ZERO_TRAJECTORY);
}

TEST(ZeroVelocityInvariant, RejectsStartupPrefixBeyondDistanceLimit)
{
  Parameters parameters;
  auto reference = test::straight_trajectory();
  auto output = reference;
  for (size_t i = 0; i < 5; ++i) {
    output.points.at(i).longitudinal_velocity_mps = 0.0F;
  }
  for (size_t i = 5; i + 1 < output.points.size(); ++i) {
    output.points.at(i).longitudinal_velocity_mps = 0.3F;
  }
  assign_time_from_start(output.points, 0.0, 1.0);

  const auto result = check_unexpected_zero_velocity(output, reference, 0.0, parameters);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, CandidateFailureReason::STARTUP_PREFIX_TOO_LONG);
}

TEST(ZeroVelocityInvariant, RejectsStartupVelocityRegressionBeforeRelease)
{
  Parameters parameters;
  parameters.max_startup_prefix_time_s = 100.0;
  auto reference = test::straight_trajectory();
  auto output = reference;
  output.points.at(0).longitudinal_velocity_mps = 0.0F;
  output.points.at(1).longitudinal_velocity_mps = 0.18F;
  output.points.at(2).longitudinal_velocity_mps = 0.05F;
  output.points.at(3).longitudinal_velocity_mps = 0.30F;
  output.points.at(4).longitudinal_velocity_mps = 0.30F;
  assign_time_from_start(output.points, 0.0, 1.0);

  const auto result = check_unexpected_zero_velocity(output, reference, 0.0, parameters);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, CandidateFailureReason::STARTUP_PROFILE_INVALID);
}

TEST(EgoAlignment, RemovesRearBufferAndRebasesAtProjectedEgoPose)
{
  auto trajectory = test::straight_trajectory();
  for (auto & point : trajectory.points) {
    point.pose.position.x -= 1.0;
  }
  const auto aligned = align_trajectory_to_ego(trajectory, test::pose(0.25, 0.0), 2.0, 3.0, 1.0);
  ASSERT_TRUE(aligned.has_value());
  ASSERT_GE(aligned->points.size(), 2U);
  EXPECT_NEAR(aligned->points.front().pose.position.x, 0.25, 1.0e-6);
  EXPECT_NEAR(aligned->points.front().pose.position.y, 0.0, 1.0e-6);
  EXPECT_NEAR(trajectory_arc_lengths(aligned->points).back(), 2.0, 1.0e-6);
  EXPECT_EQ(aligned->points.front().time_from_start.sec, 0);
  EXPECT_EQ(aligned->points.front().time_from_start.nanosec, 0U);
}

TEST(DownstreamTargetContract, DoesNotEncodeMeasuredStartupZeroAsStopPoint)
{
  Parameters parameters;
  auto target = test::straight_trajectory(0.3);
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose = test::pose(1.25, 0.0);
  odometry.twist.twist.linear.x = 0.0;

  const auto cropped = crop_downstream_target(target, odometry, test::ros_time(2.0), parameters);
  ASSERT_TRUE(cropped.has_value());
  ASSERT_GE(cropped->points.size(), 2U);
  EXPECT_GT(cropped->points.front().longitudinal_velocity_mps, 0.2F);

  const auto first_zero = autoware::motion_utils::searchZeroVelocityIndex(cropped->points);
  ASSERT_TRUE(first_zero.has_value());
  EXPECT_EQ(*first_zero, cropped->points.size() - 1);

  const auto realized =
    crop_and_retime_trajectory(target, odometry, test::ros_time(2.0), parameters);
  ASSERT_TRUE(realized.has_value());
  EXPECT_EQ(realized->points.front().longitudinal_velocity_mps, 0.0F);
}

TEST(PostSmootherValidationContract, KeepsEngageCommandSeparateFromMeasuredInitialState)
{
  auto command = test::straight_trajectory(0.3);
  command.points.at(0).longitudinal_velocity_mps = 0.25F;
  command.points.at(0).acceleration_mps2 = 0.50F;
  command.points.at(1).longitudinal_velocity_mps = 0.40311286F;
  command.points.at(1).acceleration_mps2 = 0.01329228F;
  command.points.at(2).longitudinal_velocity_mps = 0.40639690F;
  command.points.at(2).acceleration_mps2 = -0.33004105F;

  const auto timed = make_timed_validation_trajectory(command, 0.0, 1.0);

  // Validation timing starts from measured standstill, but constructing it
  // must not turn the smoother's engage command into a stop point.
  EXPECT_FLOAT_EQ(timed.points.front().longitudinal_velocity_mps, 0.25F);
  EXPECT_FLOAT_EQ(command.points.front().longitudinal_velocity_mps, 0.25F);
  EXPECT_GT(rclcpp::Duration(timed.points.at(1).time_from_start).seconds(), 0.0);

  ObjectTracker tracker{Parameters{}};
  const auto result = validate_trajectory(
    timed, test::straight_corridor(), tracker, test::ros_time(1.0), test::vehicle_info(),
    Parameters{}, false);
  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(DownstreamTargetContract, PreservesRegulatoryStop)
{
  Parameters parameters;
  auto target = test::straight_trajectory(2.0);
  for (size_t i = 12; i < target.points.size(); ++i) {
    target.points[i].longitudinal_velocity_mps = 0.0F;
  }
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose = test::pose(1.25, 0.0);

  const auto cropped = crop_downstream_target(target, odometry, test::ros_time(2.0), parameters);
  ASSERT_TRUE(cropped.has_value());
  EXPECT_TRUE(std::any_of(cropped->points.begin(), cropped->points.end(), [](const auto & point) {
    return point.longitudinal_velocity_mps == 0.0F;
  }));
}

TEST(EmergencyPolicy, ProducesStoppingTrajectoryInsteadOfStalePassThrough)
{
  Parameters parameters;
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose = test::pose(0.0, 0.0);
  odometry.twist.twist.linear.x = 3.0;
  const auto emergency = make_emergency_stop_trajectory(
    test::straight_trajectory(3.0), odometry, test::ros_time(2.0), parameters);
  ASSERT_FALSE(emergency.points.empty());
  EXPECT_EQ(emergency.points.back().longitudinal_velocity_mps, 0.0F);
  EXPECT_LE(emergency.points.at(1).longitudinal_velocity_mps, 3.0F);
}

TEST(EmergencyPolicy, StationaryEmergencyIsShortAndHasControllerMinimumPoints)
{
  Parameters parameters;
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose = test::pose(0.0, 0.0);
  odometry.twist.twist.linear.x = 0.0;
  const auto emergency = make_emergency_stop_trajectory(
    test::straight_trajectory(3.0), odometry, test::ros_time(2.0), parameters);

  ASSERT_GE(emergency.points.size(), static_cast<size_t>(parameters.emergency_min_points));
  EXPECT_LE(
    trajectory_arc_lengths(emergency.points).back(),
    parameters.emergency_stationary_length_m + 1.0e-6);
  EXPECT_TRUE(std::all_of(emergency.points.begin(), emergency.points.end(), [](const auto & point) {
    return point.longitudinal_velocity_mps == 0.0F;
  }));
}

}  // namespace autoware::fixed_route_obstacle_bypass
