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

#ifndef AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__TYPES_HPP_
#define AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__TYPES_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace autoware::fixed_route_obstacle_bypass
{

using autoware_utils_geometry::MultiPolygon2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;
using autoware_utils_geometry::Box2d;

enum class MotionState : uint8_t {
  STATIONARY = 0,
  SAME_DIRECTION = 1,
  CROSSING = 2,
  OPPOSITE_DIRECTION = 3,
  UNCERTAIN = 4,
};

enum class PlanningMode : uint8_t {
  NORMAL = 0,
  NORMAL_BYPASS = 1,
  LAST_VALID = 2,
  NO_FEASIBLE = 3,
  EMERGENCY = 4,
  WAITING_FOR_INPUT = 5,
};

enum class VelocityProfile : uint8_t {
  ACCELERATED = 0,
  NORMAL = 1,
  REDUCED = 2,
  CREEP = 3,
};

enum class PassSide : int8_t {
  RIGHT = -1,
  CENTER = 0,
  LEFT = 1,
};

enum class CandidateFailureReason : uint8_t {
  NONE = 0,
  SMOOTHER_FAILURE,
  EMPTY_FORWARD_TRAJECTORY,
  STARTUP_PREFIX_TOO_LONG,
  STARTUP_PROFILE_INVALID,
  ALL_ZERO_TRAJECTORY,
  UNEXPECTED_INTERIOR_ZERO,
  INSUFFICIENT_PROGRESS,
  LONGITUDINAL_LIMIT_FAILURE,
  LATERAL_LIMIT_FAILURE,
  COLLISION_FAILURE,
  EGO_LANE_NOT_IN_CORRIDOR,
  CORRIDOR_DISCONNECTED,
  TRAJECTORY_OUTSIDE_CORRIDOR,
  NO_LATERAL_SOLUTION,
};

struct Parameters
{
  double planning_horizon_m{120.0};
  double collision_horizon_s{12.0};
  double path_sampling_interval_m{0.5};
  double prediction_sampling_interval_s{0.2};
  double sweep_max_step_m{0.25};
  double sweep_max_yaw_step_rad{0.035};
  double route_boundary_margin_m{0.2};
  double route_gap_tolerance_m{0.03};
  double corridor_area_tolerance_m2{0.001};
  double object_collision_margin_m{0.35};
  double object_uncertainty_base_m{0.1};
  double object_uncertainty_growth_mps{0.08};
  double min_predicted_path_confidence{0.05};
  double stationary_speed_threshold_mps{0.25};
  double stationary_displacement_threshold_m{0.3};
  double crossing_speed_threshold_mps{0.35};
  double history_window_s{1.0};
  int64_t history_min_samples{3};
  double object_dropout_ttl_s{0.5};
  double uuid_association_distance_m{2.0};
  double uuid_association_size_tolerance_m{1.0};
  double cluster_longitudinal_gap_m{8.0};
  double obstacle_longitudinal_margin_m{4.0};
  double shift_prepare_distance_m{15.0};
  double shift_return_distance_m{15.0};
  std::vector<double> lateral_offsets_m{-1.6, -1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2, 1.6};
  int64_t max_geometry_candidates{32};
  double normal_target_velocity_mps{10.0};
  double reduced_velocity_ratio{0.55};
  double creep_velocity_mps{0.3};
  double accelerated_velocity_ratio{1.0};
  double minimum_progress_m{1.0};
  double progress_horizon_s{4.0};
  double zero_velocity_epsilon_mps{0.03};
  double startup_enter_velocity_mps{0.05};
  double startup_release_velocity_mps{0.20};
  int64_t startup_moving_confirmation_points{2};
  double max_startup_prefix_distance_m{2.0};
  double max_startup_prefix_time_s{5.0};
  double startup_velocity_drop_tolerance_mps{0.05};
  double endpoint_stop_tolerance_m{2.0};
  double smoother_engage_velocity_mps{0.25};
  double smoother_engage_acceleration_mps2{0.5};
  double smoother_engage_exit_ratio{0.5};
  double smoother_stop_distance_to_prohibit_engage_m{0.5};
  double max_longitudinal_acceleration_mps2{1.0};
  double min_longitudinal_acceleration_mps2{-1.0};
  double max_longitudinal_jerk_mps3{1.0};
  double min_longitudinal_jerk_mps3{-1.0};
  double max_lateral_acceleration_mps2{0.65};
  double max_steering_rate_radps{0.21};
  double nearest_distance_threshold_m{3.0};
  double nearest_yaw_threshold_rad{1.0472};
  double max_input_stamp_skew_s{0.5};
  double last_valid_hold_time_s{0.5};
  double last_valid_min_horizon_m{8.0};
  double commitment_release_distance_m{5.0};
  double emergency_deceleration_mps2{-5.0};
  double emergency_stationary_length_m{2.0};
  double emergency_braking_margin_m{2.0};
  int64_t emergency_min_points{5};
  double corridor_transition_overlap_m{5.0};
  double corridor_endpoint_buffer_m{5.0};
  double corridor_path_clip_half_width_m{10.0};
  bool debug_logging{true};
  bool exclusive_mode_verified{false};
};

struct LaneInterval
{
  int64_t lane_id{0};
  double start_s{0.0};
  double end_s{0.0};
};

struct Corridor
{
  MultiPolygon2d original;
  MultiPolygon2d center_space;
  std::vector<int64_t> ordered_lane_ids;
  std::vector<LaneInterval> lane_intervals;
  std::vector<int64_t> ego_path_lane_ids;
  int64_t ego_lane_id{0};
  std::string route_uuid;
  uint64_t route_generation{0};
};

struct StartupValidationResult
{
  bool valid{false};
  CandidateFailureReason failure_reason{CandidateFailureReason::NONE};
  size_t failure_index{0};
  double failure_s{0.0};
  double velocity_mps{0.0};
  double reference_velocity_mps{0.0};
  std::optional<size_t> startup_release_index;
  double startup_release_time_s{0.0};
};

struct OccupancyPolygon
{
  std::string track_id;
  std::string hypothesis_id;
  Polygon2d polygon;
};

struct SweptOccupancy
{
  std::string track_id;
  std::string hypothesis_id;
  Polygon2d polygon;
  Box2d envelope;
  double uncertainty_margin_m{0.0};
};

struct OccupancyTimeBin
{
  double begin_time_s{0.0};
  double end_time_s{0.0};
  std::vector<SweptOccupancy> occupancies;
};

struct OccupancyTimeline
{
  double interval_s{0.0};
  std::vector<OccupancyTimeBin> bins;
};

struct ValidationResult
{
  bool valid{false};
  bool corridor_valid{false};
  bool collision_free{false};
  bool kinematically_valid{false};
  double minimum_clearance_m{std::numeric_limits<double>::infinity()};
  std::string reason;
  std::string object_id;
  size_t trajectory_index{0};
  double trajectory_s{0.0};
  double trajectory_time_s{0.0};
  double ego_envelope_min_x{0.0};
  double ego_envelope_min_y{0.0};
  double ego_envelope_max_x{0.0};
  double ego_envelope_max_y{0.0};
  double object_envelope_min_x{0.0};
  double object_envelope_min_y{0.0};
  double object_envelope_max_x{0.0};
  double object_envelope_max_y{0.0};
  double velocity_mps{0.0};
  double reference_velocity_mps{0.0};
  std::optional<size_t> startup_release_index;
  CandidateFailureReason failure_reason{CandidateFailureReason::NONE};
  double point_containment_time_ms{0.0};
  double swept_validation_time_ms{0.0};
  size_t point_containment_count{0};
  size_t swept_footprint_count{0};
};

struct CandidateFailureContext
{
  CandidateFailureReason reason{CandidateFailureReason::NONE};
  std::string object_id;
  VelocityProfile velocity_profile{VelocityProfile::NORMAL};
  PassSide pass_side{PassSide::CENTER};
  size_t trajectory_index{0};
  double trajectory_s{0.0};
  double trajectory_time_s{0.0};
  double velocity_mps{0.0};
  double reference_velocity_mps{0.0};
  std::optional<size_t> startup_release_index;
  double startup_release_time_s{0.0};
};

struct ObstacleCluster
{
  double min_s{0.0};
  double max_s{0.0};
  std::vector<std::string> object_ids;
};

struct CandidateDescriptor
{
  std::vector<double> cluster_offsets;
  VelocityProfile velocity_profile{VelocityProfile::NORMAL};
  PassSide pass_side{PassSide::CENTER};
  double score{std::numeric_limits<double>::infinity()};
  double minimum_clearance_m{0.0};
};

inline const char * to_string(const MotionState state)
{
  switch (state) {
    case MotionState::STATIONARY:
      return "STATIONARY";
    case MotionState::SAME_DIRECTION:
      return "SAME_DIRECTION";
    case MotionState::CROSSING:
      return "CROSSING";
    case MotionState::OPPOSITE_DIRECTION:
      return "OPPOSITE_DIRECTION";
    case MotionState::UNCERTAIN:
      return "UNCERTAIN";
  }
  return "UNCERTAIN";
}

inline const char * to_string(const PlanningMode mode)
{
  switch (mode) {
    case PlanningMode::NORMAL:
      return "NORMAL";
    case PlanningMode::NORMAL_BYPASS:
      return "NORMAL_BYPASS";
    case PlanningMode::LAST_VALID:
      return "LAST_VALID";
    case PlanningMode::NO_FEASIBLE:
      return "NO_FEASIBLE_NONSTOP_TRAJECTORY";
    case PlanningMode::EMERGENCY:
      return "EMERGENCY";
    case PlanningMode::WAITING_FOR_INPUT:
      return "WAITING_FOR_INPUT";
  }
  return "UNKNOWN";
}

inline const char * to_string(const VelocityProfile profile)
{
  switch (profile) {
    case VelocityProfile::ACCELERATED:
      return "ACCELERATED";
    case VelocityProfile::NORMAL:
      return "NORMAL";
    case VelocityProfile::REDUCED:
      return "REDUCED";
    case VelocityProfile::CREEP:
      return "CREEP";
  }
  return "UNKNOWN";
}

inline const char * to_string(const PassSide side)
{
  switch (side) {
    case PassSide::RIGHT:
      return "RIGHT";
    case PassSide::CENTER:
      return "CENTER";
    case PassSide::LEFT:
      return "LEFT";
  }
  return "CENTER";
}

inline const char * to_string(const CandidateFailureReason reason)
{
  switch (reason) {
    case CandidateFailureReason::NONE:
      return "NONE";
    case CandidateFailureReason::SMOOTHER_FAILURE:
      return "SMOOTHER_FAILURE";
    case CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY:
      return "EMPTY_FORWARD_TRAJECTORY";
    case CandidateFailureReason::STARTUP_PREFIX_TOO_LONG:
      return "STARTUP_PREFIX_TOO_LONG";
    case CandidateFailureReason::STARTUP_PROFILE_INVALID:
      return "STARTUP_PROFILE_INVALID";
    case CandidateFailureReason::ALL_ZERO_TRAJECTORY:
      return "ALL_ZERO_TRAJECTORY";
    case CandidateFailureReason::UNEXPECTED_INTERIOR_ZERO:
      return "UNEXPECTED_INTERIOR_ZERO";
    case CandidateFailureReason::INSUFFICIENT_PROGRESS:
      return "INSUFFICIENT_PROGRESS";
    case CandidateFailureReason::LONGITUDINAL_LIMIT_FAILURE:
      return "LONGITUDINAL_LIMIT_FAILURE";
    case CandidateFailureReason::LATERAL_LIMIT_FAILURE:
      return "LATERAL_LIMIT_FAILURE";
    case CandidateFailureReason::COLLISION_FAILURE:
      return "COLLISION_FAILURE";
    case CandidateFailureReason::EGO_LANE_NOT_IN_CORRIDOR:
      return "EGO_LANE_NOT_IN_CORRIDOR";
    case CandidateFailureReason::CORRIDOR_DISCONNECTED:
      return "CORRIDOR_DISCONNECTED";
    case CandidateFailureReason::TRAJECTORY_OUTSIDE_CORRIDOR:
      return "TRAJECTORY_OUTSIDE_CORRIDOR";
    case CandidateFailureReason::NO_LATERAL_SOLUTION:
      return "NO_LATERAL_SOLUTION";
  }
  return "UNKNOWN";
}

}  // namespace autoware::fixed_route_obstacle_bypass

#endif  // AUTOWARE__FIXED_ROUTE_OBSTACLE_BYPASS_PLANNER__TYPES_HPP_
