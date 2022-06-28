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

#ifndef ENGAGE_TRANSITION_MANAGER__ENGAGE_TRANSITION_MANAGER_HPP_
#define ENGAGE_TRANSITION_MANAGER__ENGAGE_TRANSITION_MANAGER_HPP_

#include "engage_transition_manager/msg/engage_transition_manager_debug.hpp"

#include <engage_transition_manager/data.hpp>
#include <engage_transition_manager/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_system_msgs/msg/operation_mode.hpp>
#include <tier4_system_msgs/srv/operation_mode_request.hpp>
#include <tier4_vehicle_msgs/msg/control_mode.hpp>
#include <tier4_vehicle_msgs/srv/control_mode_request.hpp>
#include <tier4_vehicle_msgs/msg/is_autonomous_available.hpp>

namespace engage_transition_manager
{

class EngageTransitionManager : public rclcpp::Node
{
public:
  explicit EngageTransitionManager(const rclcpp::NodeOptions & options);
  ~EngageTransitionManager() = default;

private:
  rclcpp::Publisher<OperationMode>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<IsAutonomousAvailable>::SharedPtr pub_auto_available_;
  rclcpp::Publisher<EngageTransitionManagerDebug>::SharedPtr pub_debug_info_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_vehicle_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_;
  rclcpp::Service<OperationModeRequest>::SharedPtr srv_mode_change_server_;
  rclcpp::Client<OperationModeRequest>::SharedPtr srv_mode_change_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<EngageStateBase> engage_transition_manager_;

  std::shared_ptr<Data> data_;

  State updateState(const std::shared_ptr<Data> data);
  State getCurrentState() { return engage_transition_manager_->getCurrentState(); };

  EngageAcceptableParam engage_acceptable_param_;
  StableCheckParam stable_check_param_;

  bool hasDangerAcceleration();
  std::pair<bool, bool> hasDangerLateralAcceleration();
  bool checkEngageAvailable();

  void publishData();

  // update information
  void onTimer();

  void onOperationModeRequest(
    const OperationModeRequest::Request::SharedPtr request,
    const OperationModeRequest::Response::SharedPtr response);

  mutable EngageTransitionManagerDebug debug_info_;
};

}  // namespace engage_transition_manager

#endif  // ENGAGE_TRANSITION_MANAGER__ENGAGE_TRANSITION_MANAGER_HPP_
;
