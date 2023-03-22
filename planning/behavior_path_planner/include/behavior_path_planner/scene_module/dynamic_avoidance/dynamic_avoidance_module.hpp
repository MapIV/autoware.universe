// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__DYNAMIC_AVOIDANCE__DYNAMIC_AVOIDANCE_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__DYNAMIC_AVOIDANCE__DYNAMIC_AVOIDANCE_MODULE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/util/avoidance/avoidance_module_data.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;

class DynamicAvoidanceModule : public SceneModuleInterface
{
public:
#ifdef USE_OLD_ARCHITECTURE
  DynamicAvoidanceModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<DynamicAvoidanceParameters> parameters);
#else
  DynamicAvoidanceModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<DynamicAvoidanceParameters> parameters,
    std::shared_ptr<RTCInterface> & rtc_interface_left,
    std::shared_ptr<RTCInterface> & rtc_interface_right);
#endif

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  ModuleStatus updateState() override;
  ModuleStatus getNodeStatusWhileWaitingApproval() const override { return ModuleStatus::SUCCESS; }
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  BehaviorModuleOutput planWaitingApproval() override;
  void onEntry() override;
  void onExit() override;
  void updateData() override;
  void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

  void updateModuleParams(const std::shared_ptr<DynamicAvoidanceParameters> & parameters)
  {
    parameters_ = parameters;
  }

private:
  std::shared_ptr<DynamicAvoidanceParameters> parameters_;

  DynamicAvoidancePlanningData avoidance_data_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__DYNAMIC_AVOIDANCE__DYNAMIC_AVOIDANCE_MODULE_HPP_
