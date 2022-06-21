// Copyright 2022 Autoware Foundation
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

#include "engage_transition_manager/engage_transition_manager.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <algorithm>
#include <cmath>

namespace engage_transition_manager
{

using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::findNearestIndex;

EngageTransitionManager::EngageTransitionManager(const rclcpp::NodeOptions & options)
: Node("engage_transition_manager", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  engage_transition_manager_ = std::make_unique<NoneState>(this);

  pub_operation_mode_ = create_publisher<OperationMode>("engage_state", 1);
  pub_auto_available_ = create_publisher<IsAutonomousAvailable>("is_auto_available", 1);
  pub_debug_info_ = create_publisher<EngageTransitionManagerDebug>("debug_info", 1);

  sub_vehicle_kinematics_ = create_subscription<Odometry>(
    "kinematics", 1, [this](const Odometry::SharedPtr msg) { data_.kinematics = *msg; });

  sub_trajectory_ = create_subscription<Trajectory>(
    "trajectory", 1, [this](const Trajectory::SharedPtr msg) { data_.trajectory = *msg; });

  sub_control_cmd_ = create_subscription<AckermannControlCommand>(
    "control_cmd", 1,
    [this](const AckermannControlCommand::SharedPtr msg) { data_.control_cmd = *msg; });

  sub_control_mode_ = create_subscription<ControlModeReport>(
    "control_mode_report", 1,
    [this](const ControlModeReport::SharedPtr msg) { data_.current_control_mode = *msg; });

  srv_mode_change_server_ = create_service<OperationModeRequest>(
    "operation_mode_request",
    std::bind(&EngageTransitionManager::onOperationModeRequest, this, _1, _2));

  {
    auto & p = engage_acceptable_param_;
    p.dist_threshold = declare_parameter<double>("engage_acceptable_limits.dist_threshold");
    p.speed_threshold = declare_parameter<double>("engage_acceptable_limits.speed_threshold");
    p.yaw_threshold = declare_parameter<double>("engage_acceptable_limits.yaw_threshold");

    // TODO: remove
    std::cerr << "param_.dist_threshold" << p.dist_threshold << ", yaw_threshold" << p.yaw_threshold
              << ", speed_threshold" << p.speed_threshold << std::endl;
  }

  {
    auto & p = stable_check_param_;
    p.duration = declare_parameter<double>("stable_check.duration");
    p.dist_threshold = declare_parameter<double>("stable_check.dist_threshold");
    p.speed_threshold = declare_parameter<double>("stable_check.speed_threshold");
    p.yaw_threshold = declare_parameter<double>("stable_check.yaw_threshold");

    // TODO: remove
    std::cerr << "stable_check_param_.duration" << p.duration << "dist_threshold"
              << p.dist_threshold << ", yaw_threshold" << p.yaw_threshold << ", speed_threshold"
              << p.speed_threshold << std::endl;
  }

  {
    const auto hz = declare_parameter<double>("frequency_hz");
    const auto period_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&EngageTransitionManager::onTimer, this));
  }
}

void EngageTransitionManager::onOperationModeRequest(
  const OperationModeRequest::Request::SharedPtr request,
  const OperationModeRequest::Response::SharedPtr response)
{
  // TODO: remove
  RCLCPP_INFO_STREAM(get_logger(), "onOperationModeRequest callback()");

  const auto req_state = toEnum(request->mode);

  // invalid case
  if (req_state == State::TRANSITION_TO_AUTO) {
    RCLCPP_WARN(
      get_logger(), "mode change to TRANSITION_TO_AUTO is not supported. Request ignored.");
    response->success = false;
    return;
  }

  data_.requested_state = toEnum(request->mode);

  const auto state = updateState(data_);

  // write success/failure condition one by one for the state transition result.
  if (req_state == state) {
    response->success = true;
    return;
  }
  if (isAuto(req_state) && state == State::TRANSITION_TO_AUTO) {
    response->success = true;
    return;
  }

  // not satisfy any success conditions.
  response->success = false;
  return;
}

void EngageTransitionManager::onTimer()
{
  data_.is_auto_available = checkEngageAvailable();

  updateState(data_);

  publishData();

  // TODO: remove
  RCLCPP_INFO_STREAM(
    get_logger(), "Timer: engage_available: "
                    << (data_.is_auto_available ? "True" : "False")
                    << ", requested_state: " << toStr(data_.requested_state)
                    << ", current state: " << toStr(engage_transition_manager_->getCurrentState()));
}

void EngageTransitionManager::publishData()
{
  const auto time = now();

  OperationMode mode;
  mode.stamp = time;
  mode.mode = toMsg(engage_transition_manager_->getCurrentState());
  pub_operation_mode_->publish(mode);

  IsAutonomousAvailable msg;
  msg.stamp = time;
  msg.is_autonomous_available = data_.is_auto_available;
  pub_auto_available_->publish(msg);

  debug_info_.stamp = time;
  pub_debug_info_->publish(debug_info_);
}

bool EngageTransitionManager::hasDangerAcceleration()
{
  const bool is_stopping = std::abs(data_.kinematics.twist.twist.linear.x) < 0.01;
  if (is_stopping) {
    return false;  // any acceleration is ok when stopped
  }

  const bool has_large_acc = std::abs(data_.control_cmd.longitudinal.acceleration) >
                             engage_acceptable_param_.large_acc_threshold;
  return has_large_acc;
}

bool EngageTransitionManager::checkEngageAvailable()
{
  constexpr auto dist_max = 5.0;
  constexpr auto yaw_max = M_PI_4;

  if (data_.trajectory.points.size() < 2) {
    RCLCPP_WARN(get_logger(), "Engage unavailable: trajectory size must be > 2");
    debug_info_ = EngageTransitionManagerDebug{};  // all false
    return false;
  }

  const auto closest_idx =
    findNearestIndex(data_.trajectory.points, data_.kinematics.pose.pose, dist_max, yaw_max);
  if (!closest_idx) {
    RCLCPP_INFO(get_logger(), "Engage unavailable: closest point not found");
    debug_info_ = EngageTransitionManagerDebug{};  // all false
    return false;                                  // closest trajectory point not found.
  }
  const auto closest_point = data_.trajectory.points.at(*closest_idx);
  debug_info_.trajectory_available_ok = true;

  // No engagement is lateral control error is large
  const auto lateral_deviation = calcDistance2d(closest_point.pose, data_.kinematics.pose.pose);
  const bool lateral_deviation_ok = lateral_deviation < engage_acceptable_param_.dist_threshold;

  // No engagement is yaw control error is large
  const auto yaw_deviation = calcYawDeviation(closest_point.pose, data_.kinematics.pose.pose);
  const bool yaw_deviation_ok = yaw_deviation < engage_acceptable_param_.yaw_threshold;

  // No engagement if speed control error is large
  const auto speed_deviation =
    std::abs(closest_point.longitudinal_velocity_mps - data_.kinematics.twist.twist.linear.x);
  const bool speed_deviation_ok = speed_deviation < engage_acceptable_param_.speed_threshold;

  // No engagement if the vehicle is moving but the target speed is zero.
  const bool no_stop_ok =
    !(std::abs(data_.kinematics.twist.twist.linear.x) > 0.1 &&
      std::abs(data_.control_cmd.longitudinal.speed) < 0.01);

  // No engagement if the large acceleration is commanded.
  const bool no_large_acceleration_ok = !hasDangerAcceleration();

  // No engagement if a stop is expected within a certain period of time
  // TODO: write me
  // ...

  debug_info_.lateral_deviation_ok = lateral_deviation_ok;
  debug_info_.yaw_deviation_ok = yaw_deviation_ok;
  debug_info_.speed_deviation_ok = speed_deviation_ok;
  debug_info_.no_stop_ok = no_stop_ok;
  debug_info_.no_large_acceleration_ok = no_large_acceleration_ok;

  const bool is_all_ok = lateral_deviation_ok && yaw_deviation_ok && speed_deviation_ok &&
                         no_stop_ok && no_large_acceleration_ok;

  return is_all_ok;
}

State EngageTransitionManager::updateState(const Data & data)
{
  const auto current_state = engage_transition_manager_->getCurrentState();

  engage_transition_manager_->setData(data);
  const auto next_state = engage_transition_manager_->update();

  // no state change
  if (next_state == current_state) {
    return current_state;
  }

  // transit state
  switch (next_state) {
    case State::STOP:
      engage_transition_manager_ = std::make_unique<NoneState>(this);
      break;
    case State::REMOTE_OPERATOR:
      engage_transition_manager_ = std::make_unique<RemoteState>(this);
      break;
    case State::MANUAL_DIRECT:
      engage_transition_manager_ = std::make_unique<DirectState>(this);
      break;
    case State::LOCAL_OPERATOR:
      engage_transition_manager_ = std::make_unique<LocalState>(this);
      break;
    case State::TRANSITION_TO_AUTO:
      engage_transition_manager_ = std::make_unique<TransitionState>(this);
      break;
    case State::AUTONOMOUS:
      engage_transition_manager_ = std::make_unique<AutonomousState>(this);
      break;
  }

  if (next_state != engage_transition_manager_->getCurrentState()) {
    throw std::runtime_error("engage_transition_manager: unexpected state change!");
  }

  return engage_transition_manager_->getCurrentState();
}

}  // namespace engage_transition_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(engage_transition_manager::EngageTransitionManager)
