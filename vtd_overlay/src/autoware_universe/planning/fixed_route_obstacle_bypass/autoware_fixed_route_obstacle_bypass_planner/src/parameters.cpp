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

#include "autoware/fixed_route_obstacle_bypass_planner/parameters.hpp"

#include <string>

namespace autoware::fixed_route_obstacle_bypass
{

Parameters declare_parameters(rclcpp::Node & node)
{
  Parameters p;
  const std::string ns = "fixed_route_bypass.";
  p.planning_horizon_m =
    node.declare_parameter<double>(ns + "planning_horizon_m", p.planning_horizon_m);
  p.collision_horizon_s =
    node.declare_parameter<double>(ns + "collision_horizon_s", p.collision_horizon_s);
  p.path_sampling_interval_m =
    node.declare_parameter<double>(ns + "path_sampling_interval_m", p.path_sampling_interval_m);
  p.prediction_sampling_interval_s = node.declare_parameter<double>(
    ns + "prediction_sampling_interval_s", p.prediction_sampling_interval_s);
  p.sweep_max_step_m = node.declare_parameter<double>(ns + "sweep_max_step_m", p.sweep_max_step_m);
  p.sweep_max_yaw_step_rad =
    node.declare_parameter<double>(ns + "sweep_max_yaw_step_rad", p.sweep_max_yaw_step_rad);
  p.route_boundary_margin_m =
    node.declare_parameter<double>(ns + "route_boundary_margin_m", p.route_boundary_margin_m);
  p.route_gap_tolerance_m =
    node.declare_parameter<double>(ns + "route_gap_tolerance_m", p.route_gap_tolerance_m);
  p.corridor_area_tolerance_m2 =
    node.declare_parameter<double>(ns + "corridor_area_tolerance_m2", p.corridor_area_tolerance_m2);
  p.object_collision_margin_m =
    node.declare_parameter<double>(ns + "object_collision_margin_m", p.object_collision_margin_m);
  p.object_uncertainty_base_m =
    node.declare_parameter<double>(ns + "object_uncertainty_base_m", p.object_uncertainty_base_m);
  p.object_uncertainty_growth_mps = node.declare_parameter<double>(
    ns + "object_uncertainty_growth_mps", p.object_uncertainty_growth_mps);
  p.min_predicted_path_confidence = node.declare_parameter<double>(
    ns + "min_predicted_path_confidence", p.min_predicted_path_confidence);
  p.stationary_speed_threshold_mps = node.declare_parameter<double>(
    ns + "stationary_speed_threshold_mps", p.stationary_speed_threshold_mps);
  p.stationary_displacement_threshold_m = node.declare_parameter<double>(
    ns + "stationary_displacement_threshold_m", p.stationary_displacement_threshold_m);
  p.crossing_speed_threshold_mps = node.declare_parameter<double>(
    ns + "crossing_speed_threshold_mps", p.crossing_speed_threshold_mps);
  p.history_window_s = node.declare_parameter<double>(ns + "history_window_s", p.history_window_s);
  p.history_min_samples =
    node.declare_parameter<int64_t>(ns + "history_min_samples", p.history_min_samples);
  p.object_dropout_ttl_s =
    node.declare_parameter<double>(ns + "object_dropout_ttl_s", p.object_dropout_ttl_s);
  p.uuid_association_distance_m = node.declare_parameter<double>(
    ns + "uuid_association_distance_m", p.uuid_association_distance_m);
  p.uuid_association_size_tolerance_m = node.declare_parameter<double>(
    ns + "uuid_association_size_tolerance_m", p.uuid_association_size_tolerance_m);
  p.cluster_longitudinal_gap_m =
    node.declare_parameter<double>(ns + "cluster_longitudinal_gap_m", p.cluster_longitudinal_gap_m);
  p.obstacle_longitudinal_margin_m = node.declare_parameter<double>(
    ns + "obstacle_longitudinal_margin_m", p.obstacle_longitudinal_margin_m);
  p.shift_prepare_distance_m =
    node.declare_parameter<double>(ns + "shift_prepare_distance_m", p.shift_prepare_distance_m);
  p.shift_return_distance_m =
    node.declare_parameter<double>(ns + "shift_return_distance_m", p.shift_return_distance_m);
  p.lateral_offsets_m =
    node.declare_parameter<std::vector<double>>(ns + "lateral_offsets_m", p.lateral_offsets_m);
  p.max_geometry_candidates =
    node.declare_parameter<int64_t>(ns + "max_geometry_candidates", p.max_geometry_candidates);
  p.normal_target_velocity_mps =
    node.declare_parameter<double>(ns + "normal_target_velocity_mps", p.normal_target_velocity_mps);
  p.reduced_velocity_ratio =
    node.declare_parameter<double>(ns + "reduced_velocity_ratio", p.reduced_velocity_ratio);
  p.creep_velocity_mps =
    node.declare_parameter<double>(ns + "creep_velocity_mps", p.creep_velocity_mps);
  p.accelerated_velocity_ratio =
    node.declare_parameter<double>(ns + "accelerated_velocity_ratio", p.accelerated_velocity_ratio);
  p.minimum_progress_m =
    node.declare_parameter<double>(ns + "minimum_progress_m", p.minimum_progress_m);
  p.progress_horizon_s =
    node.declare_parameter<double>(ns + "progress_horizon_s", p.progress_horizon_s);
  p.zero_velocity_epsilon_mps =
    node.declare_parameter<double>(ns + "zero_velocity_epsilon_mps", p.zero_velocity_epsilon_mps);
  p.startup_enter_velocity_mps =
    node.declare_parameter<double>(ns + "startup_enter_velocity_mps", p.startup_enter_velocity_mps);
  p.startup_release_velocity_mps = node.declare_parameter<double>(
    ns + "startup_release_velocity_mps", p.startup_release_velocity_mps);
  p.startup_moving_confirmation_points = node.declare_parameter<int64_t>(
    ns + "startup_moving_confirmation_points", p.startup_moving_confirmation_points);
  p.max_startup_prefix_distance_m = node.declare_parameter<double>(
    ns + "max_startup_prefix_distance_m", p.max_startup_prefix_distance_m);
  p.max_startup_prefix_time_s =
    node.declare_parameter<double>(ns + "max_startup_prefix_time_s", p.max_startup_prefix_time_s);
  p.startup_velocity_drop_tolerance_mps = node.declare_parameter<double>(
    ns + "startup_velocity_drop_tolerance_mps", p.startup_velocity_drop_tolerance_mps);
  p.endpoint_stop_tolerance_m =
    node.declare_parameter<double>(ns + "endpoint_stop_tolerance_m", p.endpoint_stop_tolerance_m);
  // These un-namespaced parameters intentionally use the same keys as the
  // downstream Autoware velocity-smoother node.  Both nodes load the same
  // velocity_smoother.param.yaml, so the planner's collision prediction uses
  // the same standstill engage initial condition as the trajectory that is
  // ultimately sent to control.
  p.smoother_engage_velocity_mps =
    node.declare_parameter<double>("engage_velocity", p.smoother_engage_velocity_mps);
  p.smoother_engage_acceleration_mps2 = node.declare_parameter<double>(
    "engage_acceleration", p.smoother_engage_acceleration_mps2);
  p.smoother_engage_exit_ratio =
    node.declare_parameter<double>("engage_exit_ratio", p.smoother_engage_exit_ratio);
  p.smoother_stop_distance_to_prohibit_engage_m = node.declare_parameter<double>(
    "stop_dist_to_prohibit_engage", p.smoother_stop_distance_to_prohibit_engage_m);
  p.max_longitudinal_acceleration_mps2 = node.declare_parameter<double>(
    ns + "max_longitudinal_acceleration_mps2", p.max_longitudinal_acceleration_mps2);
  p.min_longitudinal_acceleration_mps2 = node.declare_parameter<double>(
    ns + "min_longitudinal_acceleration_mps2", p.min_longitudinal_acceleration_mps2);
  p.max_longitudinal_jerk_mps3 =
    node.declare_parameter<double>(ns + "max_longitudinal_jerk_mps3", p.max_longitudinal_jerk_mps3);
  p.min_longitudinal_jerk_mps3 =
    node.declare_parameter<double>(ns + "min_longitudinal_jerk_mps3", p.min_longitudinal_jerk_mps3);
  p.max_lateral_acceleration_mps2 = node.declare_parameter<double>(
    ns + "max_lateral_acceleration_mps2", p.max_lateral_acceleration_mps2);
  p.max_steering_rate_radps =
    node.declare_parameter<double>(ns + "max_steering_rate_radps", p.max_steering_rate_radps);
  p.nearest_distance_threshold_m = node.declare_parameter<double>(
    ns + "nearest_distance_threshold_m", p.nearest_distance_threshold_m);
  p.nearest_yaw_threshold_rad =
    node.declare_parameter<double>(ns + "nearest_yaw_threshold_rad", p.nearest_yaw_threshold_rad);
  p.max_input_stamp_skew_s =
    node.declare_parameter<double>(ns + "max_input_stamp_skew_s", p.max_input_stamp_skew_s);
  p.last_valid_hold_time_s =
    node.declare_parameter<double>(ns + "last_valid_hold_time_s", p.last_valid_hold_time_s);
  p.last_valid_min_horizon_m =
    node.declare_parameter<double>(ns + "last_valid_min_horizon_m", p.last_valid_min_horizon_m);
  p.commitment_release_distance_m = node.declare_parameter<double>(
    ns + "commitment_release_distance_m", p.commitment_release_distance_m);
  p.emergency_deceleration_mps2 = node.declare_parameter<double>(
    ns + "emergency_deceleration_mps2", p.emergency_deceleration_mps2);
  p.emergency_stationary_length_m = node.declare_parameter<double>(
    ns + "emergency_stationary_length_m", p.emergency_stationary_length_m);
  p.emergency_braking_margin_m =
    node.declare_parameter<double>(ns + "emergency_braking_margin_m", p.emergency_braking_margin_m);
  p.emergency_min_points =
    node.declare_parameter<int64_t>(ns + "emergency_min_points", p.emergency_min_points);
  p.corridor_transition_overlap_m = node.declare_parameter<double>(
    ns + "corridor_transition_overlap_m", p.corridor_transition_overlap_m);
  p.corridor_endpoint_buffer_m =
    node.declare_parameter<double>(ns + "corridor_endpoint_buffer_m", p.corridor_endpoint_buffer_m);
  p.corridor_path_clip_half_width_m = node.declare_parameter<double>(
    ns + "corridor_path_clip_half_width_m", p.corridor_path_clip_half_width_m);
  p.debug_logging = node.declare_parameter<bool>(ns + "debug_logging", p.debug_logging);
  p.exclusive_mode_verified =
    node.declare_parameter<bool>(ns + "exclusive_mode_verified", p.exclusive_mode_verified);
  return p;
}

}  // namespace autoware::fixed_route_obstacle_bypass
