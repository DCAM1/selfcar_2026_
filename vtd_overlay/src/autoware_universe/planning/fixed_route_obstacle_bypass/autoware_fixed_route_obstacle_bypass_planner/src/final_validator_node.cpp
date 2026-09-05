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

#include "autoware/fixed_route_obstacle_bypass_planner/final_validator_node.hpp"

#include "autoware/fixed_route_obstacle_bypass_planner/parameters.hpp"
#include "autoware/fixed_route_obstacle_bypass_planner/trajectory.hpp"

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::fixed_route_obstacle_bypass
{
namespace
{
using autoware_planning_msgs::msg::Trajectory;

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

FixedRouteFinalTrajectoryValidatorNode::FixedRouteFinalTrajectoryValidatorNode(
  const rclcpp::NodeOptions & options)
: Node("fixed_route_final_trajectory_validator", options), object_tracker_(parameters_)
{
  parameters_ = autoware::fixed_route_obstacle_bypass::declare_parameters(*this);
  object_tracker_.set_parameters(parameters_);
  if (!parameters_.exclusive_mode_verified) {
    throw std::runtime_error(
      "fixed-route final validator must only run with "
      "the mutually-exclusive bypass launch");
  }
  if (
    parameters_.prediction_sampling_interval_s <= 0.0 ||
    parameters_.max_longitudinal_acceleration_mps2 <= 0.0 ||
    parameters_.startup_release_velocity_mps <= parameters_.zero_velocity_epsilon_mps ||
    parameters_.startup_enter_velocity_mps < parameters_.zero_velocity_epsilon_mps ||
    parameters_.startup_moving_confirmation_points < 1 ||
    parameters_.max_startup_prefix_distance_m <= 0.0 ||
    parameters_.max_startup_prefix_time_s <= 0.0 || parameters_.emergency_min_points < 2 ||
    parameters_.emergency_stationary_length_m <= 0.0 ||
    parameters_.corridor_path_clip_half_width_m <= 0.0 ||
    parameters_.corridor_transition_overlap_m < 0.0) {
    throw std::invalid_argument("invalid fixed-route final-validator parameters");
  }
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", rclcpp::QoS{1});
  diagnostics_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/output/diagnostics", rclcpp::QoS{1});

  input_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  validation_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions input_subscription_options;
  input_subscription_options.callback_group = input_callback_group_;
  rclcpp::SubscriptionOptions validation_subscription_options;
  validation_subscription_options.callback_group = validation_callback_group_;

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
        regulatory_references_by_stamp_.clear();
        planner_states_by_stamp_.clear();
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
    },
    input_subscription_options);
  reference_sub_ = create_subscription<Trajectory>(
    "~/input/regulatory_reference", rclcpp::QoS{1}, [this](const Trajectory::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      regulatory_reference_ = msg;
      const auto stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME).nanoseconds();
      if (stamp > 0) {
        regulatory_references_by_stamp_[stamp] = *msg;
        constexpr size_t max_reference_history = 100;
        while (regulatory_references_by_stamp_.size() > max_reference_history) {
          regulatory_references_by_stamp_.erase(regulatory_references_by_stamp_.begin());
        }
      }
    },
    input_subscription_options);
  planner_state_sub_ = create_subscription<autoware_internal_debug_msgs::msg::StringStamped>(
    "~/input/planner_state", durable_qos,
    [this](const autoware_internal_debug_msgs::msg::StringStamped::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto stamp = rclcpp::Time(msg->stamp, RCL_ROS_TIME).nanoseconds();
      planner_states_by_stamp_[stamp] = msg->data;
      constexpr size_t max_state_history = 100;
      while (planner_states_by_stamp_.size() > max_state_history) {
        planner_states_by_stamp_.erase(planner_states_by_stamp_.begin());
      }
    },
    input_subscription_options);
  trajectory_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    std::bind(&FixedRouteFinalTrajectoryValidatorNode::on_trajectory, this, std::placeholders::_1),
    validation_subscription_options);

  RCLCPP_INFO(get_logger(), "post-smoother fixed-route swept-occupancy validation enabled");
}

void FixedRouteFinalTrajectoryValidatorNode::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  const auto callback_started = std::chrono::steady_clock::now();
  const auto planning_time = now();
  ValidationResult result;
  std::optional<nav_msgs::msg::Odometry> odometry;
  std::optional<Trajectory> regulatory_reference;
  std::optional<Corridor> corridor;
  ObjectTracker object_tracker_snapshot{parameters_};
  bool planner_is_emergency = false;
  std::optional<Trajectory> last_valid_target;
  rclcpp::Time last_valid_time{0, 0, RCL_ROS_TIME};
  uint64_t last_valid_route_generation = 0;
  std::string input_error;
  CandidateFailureReason input_failure = CandidateFailureReason::NONE;

  // Snapshot all mutable inputs, then release the lock before the expensive
  // swept-polygon validation. With the final validator's multi-threaded
  // executor this lets map/path/object/odometry callbacks remain current while
  // one trajectory is being checked.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    object_tracker_.prune(planning_time);
    if (odometry_) {
      odometry = *odometry_;
    }
    const bool object_stream_fresh =
      objects_received_ &&
      (planning_time - last_objects_received_time_).seconds() <=
        std::max(parameters_.object_dropout_ttl_s, parameters_.max_input_stamp_skew_s);
    if (!odometry_ || !regulatory_reference_ || !object_stream_fresh ||
        !corridor_builder_.ready()) {
      input_error = "post-smoother validator inputs are not ready";
    } else if (!corridor_builder_.input_is_current(
                 msg->header.stamp, parameters_.max_input_stamp_skew_s, &input_error, false)) {
      // input_error is populated by the corridor builder. Final trajectories
      // are expected to lag the latest PathWithLaneId while validation runs,
      // so only active-route/path-generation consistency is required here.
    } else {
      const rclcpp::Time trajectory_stamp(msg->header.stamp, RCL_ROS_TIME);
      const Trajectory * selected_reference = regulatory_reference_.get();
      // Prefer the reference from the same planning cycle. Validation is
      // intentionally heavier than the upstream publishers, however, and the
      // exact sample can be dropped by latest-value QoS while validation is
      // running. The newest reference from the same route generation is the
      // correct spatial/regulatory fallback.
      if (trajectory_stamp.nanoseconds() > 0) {
        const auto exact_reference =
          regulatory_references_by_stamp_.find(trajectory_stamp.nanoseconds());
        if (exact_reference != regulatory_references_by_stamp_.end()) {
          selected_reference = &exact_reference->second;
        }
      }
      regulatory_reference = *selected_reference;

      const double vehicle_half_width = std::max(
        std::abs(vehicle_info_.min_lateral_offset_m),
        std::abs(vehicle_info_.max_lateral_offset_m));
      std::string corridor_error;
      CandidateFailureReason corridor_failure = CandidateFailureReason::NONE;
      corridor = corridor_builder_.build(
        odometry->pose.pose, parameters_.planning_horizon_m, vehicle_half_width, parameters_,
        &corridor_error, &corridor_failure);
      if (!corridor) {
        input_error = corridor_error;
        input_failure = corridor_failure;
      } else {
        object_tracker_snapshot = object_tracker_;
        const auto planner_state =
          planner_states_by_stamp_.find(trajectory_stamp.nanoseconds());
        planner_is_emergency =
          planner_state != planner_states_by_stamp_.end() &&
          planner_state->second == to_string(PlanningMode::EMERGENCY);
        last_valid_target = last_valid_target_;
        last_valid_time = last_valid_time_;
        last_valid_route_generation = last_valid_route_generation_;
      }
    }
  }
  const auto snapshot_completed = std::chrono::steady_clock::now();

  const auto handle_input_failure = [&](
                                      const std::string & reason,
                                      const CandidateFailureReason failure_reason =
                                        CandidateFailureReason::NONE,
                                      const Corridor * failure_corridor = nullptr) {
    result.reason = reason;
    result.failure_reason = failure_reason;
    if (odometry && msg->points.size() >= 2) {
      auto emergency = make_emergency_stop_trajectory(*msg, *odometry, planning_time, parameters_);
      trajectory_pub_->publish(emergency);
      publish_diagnostic(
        PlanningMode::EMERGENCY, result, "input failure; published emergency guard stop",
        failure_corridor);
    } else {
      publish_diagnostic(
        PlanningMode::WAITING_FOR_INPUT, result, "no trajectory published", failure_corridor);
    }
  };
  if (!input_error.empty() || !odometry || !regulatory_reference || !corridor) {
    handle_input_failure(
      input_error.empty() ? "post-smoother validator inputs are not ready" : input_error,
      input_failure, corridor ? &*corridor : nullptr);
    return;
  }

  const auto aligned_trajectory = align_trajectory_to_ego(
    *msg, odometry->pose.pose, parameters_.planning_horizon_m,
    parameters_.nearest_distance_threshold_m, parameters_.nearest_yaw_threshold_rad);
  if (!aligned_trajectory) {
    handle_input_failure(
      "EMPTY_FORWARD_TRAJECTORY: failed to align "
      "post-smoother output to ego",
      CandidateFailureReason::EMPTY_FORWARD_TRAJECTORY, &*corridor);
    return;
  }
  // The global velocity smoother's output is the command-side source of
  // truth. In particular, at standstill it deliberately publishes its engage
  // velocity/acceleration at the first point. Keep that message unchanged for
  // control and create a separate, timed copy for collision prediction.
  Trajectory output_trajectory = *aligned_trajectory;
  Trajectory timed_trajectory = make_timed_validation_trajectory(
    output_trajectory, odometry->twist.twist.linear.x,
    parameters_.max_longitudinal_acceleration_mps2);
  const auto preparation_completed = std::chrono::steady_clock::now();
  if (planner_is_emergency) {
    result = validate_emergency_trajectory(
      timed_trajectory, *corridor, object_tracker_snapshot, planning_time, vehicle_info_,
      parameters_);
    if (result.valid) {
      trajectory_pub_->publish(output_trajectory);
      publish_diagnostic(
        PlanningMode::EMERGENCY, result, "published validated emergency braking trajectory",
        &*corridor);
    } else {
      const auto & emergency_reference =
        regulatory_reference->points.empty() ? *msg : *regulatory_reference;
      auto emergency =
        make_emergency_stop_trajectory(emergency_reference, *odometry, planning_time, parameters_);
      trajectory_pub_->publish(emergency);
      publish_diagnostic(
        PlanningMode::EMERGENCY, result,
        "emergency input invalid; published fresh emergency "
        "guard stop, never stale trajectory",
        &*corridor);
    }
    return;
  }

  const auto startup_validation = check_unexpected_zero_velocity(
    timed_trajectory, *regulatory_reference, odometry->twist.twist.linear.x, parameters_);
  const auto startup_validation_completed = std::chrono::steady_clock::now();
  if (!startup_validation.valid) {
    result.reason = to_string(startup_validation.failure_reason);
    result.failure_reason = startup_validation.failure_reason;
    result.trajectory_index = startup_validation.failure_index;
    result.trajectory_s = startup_validation.failure_s;
    result.velocity_mps = startup_validation.velocity_mps;
    result.reference_velocity_mps = startup_validation.reference_velocity_mps;
    result.startup_release_index = startup_validation.startup_release_index;
  } else {
    // The upstream global velocity smoother is the authoritative
    // longitudinal optimizer. Re-differentiating its spatial velocity profile
    // over a locally reconstructed time axis is not an equivalent jerk check
    // (the smoother uses soft pseudo-jerk constraints) and rejects valid
    // engage profiles. This final stage therefore rechecks the properties that
    // can change with final timing: corridor containment, swept collision,
    // zero-velocity invariants, and progress.
    result = validate_trajectory(
      timed_trajectory, *corridor, object_tracker_snapshot, planning_time, vehicle_info_,
      parameters_, false);
    result.startup_release_index = startup_validation.startup_release_index;
    const auto arc = trajectory_arc_lengths(timed_trajectory.points);
    if (result.trajectory_index < arc.size()) {
      result.trajectory_s = arc[result.trajectory_index];
      result.velocity_mps =
        std::abs(timed_trajectory.points[result.trajectory_index].longitudinal_velocity_mps);
    }
    if (
      result.valid &&
      !has_nonterminal_regulatory_stop(
        *regulatory_reference, timed_trajectory.points.front().pose, parameters_) &&
      progress_at_time(timed_trajectory, parameters_.progress_horizon_s) <
        parameters_.minimum_progress_m) {
      result.valid = false;
      result.reason = to_string(CandidateFailureReason::INSUFFICIENT_PROGRESS);
      result.failure_reason = CandidateFailureReason::INSUFFICIENT_PROGRESS;
      result.trajectory_s = progress_at_time(timed_trajectory, parameters_.progress_horizon_s);
    }
  }
  const auto validation_completed = std::chrono::steady_clock::now();
  const auto milliseconds = [](const auto begin, const auto end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
  };
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "validation timing [ms]: snapshot=%.1f preparation=%.1f startup=%.1f "
    "point_containment=%.1f/%zu swept=%.1f/%zu post=%.1f total=%.1f",
    milliseconds(callback_started, snapshot_completed),
    milliseconds(snapshot_completed, preparation_completed),
    milliseconds(preparation_completed, startup_validation_completed),
    result.point_containment_time_ms, result.point_containment_count,
    result.swept_validation_time_ms, result.swept_footprint_count,
    milliseconds(startup_validation_completed, validation_completed),
    milliseconds(callback_started, validation_completed));

  if (result.valid) {
    output_trajectory.header.stamp = msg->header.stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (corridor_builder_.route_generation() == corridor->route_generation) {
        last_valid_target_ = output_trajectory;
        last_valid_time_ = planning_time;
        last_valid_route_generation_ = corridor->route_generation;
      }
    }
    trajectory_pub_->publish(output_trajectory);
    publish_diagnostic(
      planner_is_emergency ? PlanningMode::EMERGENCY : PlanningMode::NORMAL, result,
      "published current post-smoother trajectory", &*corridor);
    return;
  }

  if (
    last_valid_target && last_valid_route_generation == corridor->route_generation &&
    (planning_time - last_valid_time).seconds() <= parameters_.last_valid_hold_time_s) {
    const auto remaining_target =
      crop_downstream_target(*last_valid_target, *odometry, planning_time, parameters_);
    if (remaining_target) {
      const auto remaining = make_timed_validation_trajectory(
        *remaining_target, odometry->twist.twist.linear.x,
        parameters_.max_longitudinal_acceleration_mps2);
      const auto startup_validation = check_unexpected_zero_velocity(
        remaining, *regulatory_reference, odometry->twist.twist.linear.x, parameters_);
      const bool progress_valid =
        has_nonterminal_regulatory_stop(
          *regulatory_reference, remaining.points.front().pose, parameters_) ||
        progress_at_time(remaining, parameters_.progress_horizon_s) >=
          parameters_.minimum_progress_m;
      const auto last_result = validate_trajectory(
        remaining, *corridor, object_tracker_snapshot, planning_time, vehicle_info_, parameters_,
        false);
      if (startup_validation.valid && progress_valid && last_result.valid) {
        trajectory_pub_->publish(*remaining_target);
        publish_diagnostic(
          PlanningMode::LAST_VALID, result,
          "current output rejected; published freshly "
          "revalidated last-valid remainder",
          &*corridor);
        return;
      }
    }
  }

  const auto & emergency_reference =
    regulatory_reference->points.empty() ? *msg : *regulatory_reference;
  auto emergency =
    make_emergency_stop_trajectory(emergency_reference, *odometry, planning_time, parameters_);
  trajectory_pub_->publish(emergency);
  publish_diagnostic(
    PlanningMode::EMERGENCY, result,
    "current and last-valid trajectories rejected; published "
    "emergency guard stop",
    &*corridor);
}

void FixedRouteFinalTrajectoryValidatorNode::publish_diagnostic(
  const PlanningMode mode, const ValidationResult & result, const std::string & action,
  const Corridor * corridor)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "planning/fixed_route_final_trajectory_validator";
  status.hardware_id = "planning";
  status.level = mode == PlanningMode::EMERGENCY
                   ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                   : (mode == PlanningMode::LAST_VALID || mode == PlanningMode::WAITING_FOR_INPUT
                        ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                        : diagnostic_msgs::msg::DiagnosticStatus::OK);
  status.message = result.reason.empty() ? action : result.reason;
  status.values.push_back(key_value("mode", to_string(mode)));
  status.values.push_back(key_value("action", action));
  status.values.push_back(key_value("collision_free", result.collision_free ? "true" : "false"));
  status.values.push_back(key_value("corridor_valid", result.corridor_valid ? "true" : "false"));
  status.values.push_back(
    key_value("kinematically_valid", result.kinematically_valid ? "true" : "false"));
  status.values.push_back(key_value("object_id", result.object_id));
  status.values.push_back(key_value("failure_reason", to_string(result.failure_reason)));
  status.values.push_back(key_value("failure_index", std::to_string(result.trajectory_index)));
  status.values.push_back(key_value("failure_s", std::to_string(result.trajectory_s)));
  status.values.push_back(key_value("failure_time_s", std::to_string(result.trajectory_time_s)));
  status.values.push_back(key_value("failure_velocity", std::to_string(result.velocity_mps)));
  status.values.push_back(
    key_value("failure_reference_velocity", std::to_string(result.reference_velocity_mps)));
  status.values.push_back(key_value(
    "startup_release_index", result.startup_release_index
                               ? std::to_string(*result.startup_release_index)
                               : std::string{"NONE"}));
  status.values.push_back(key_value("route_uuid", corridor ? corridor->route_uuid : std::string{}));
  if (corridor) {
    status.values.push_back(key_value("ego_index", "0"));
    status.values.push_back(
      key_value("corridor_lane_ids", lane_ids_to_string(corridor->ordered_lane_ids)));
    status.values.push_back(
      key_value("path_lane_ids_at_ego", lane_ids_to_string(corridor->ego_path_lane_ids)));
    status.values.push_back(key_value("ego_lane_id", std::to_string(corridor->ego_lane_id)));
  }
  const auto log_message = status.message;
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);

  if (mode == PlanningMode::EMERGENCY) {
    RCLCPP_ERROR(get_logger(), "%s: %s", action.c_str(), log_message.c_str());
  } else if (mode == PlanningMode::LAST_VALID || mode == PlanningMode::WAITING_FOR_INPUT) {
    RCLCPP_WARN(get_logger(), "%s: %s", action.c_str(), log_message.c_str());
  }
}

}  // namespace autoware::fixed_route_obstacle_bypass

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::fixed_route_obstacle_bypass::FixedRouteFinalTrajectoryValidatorNode)
