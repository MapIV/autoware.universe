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

#ifndef BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
#define BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_

#include "behavior_path_planner/behavior_tree_manager.hpp"
#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/scene_module/lane_change/external_request_lane_change_module.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/scene_module/lane_following/lane_following_module.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_module.hpp"
#include "behavior_path_planner/scene_module/pull_over/pull_over_module.hpp"
#include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"

#include <route_handler/route_handler.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/approval.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_factor.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/path_change_module.hpp>
#include <tier4_planning_msgs/msg/path_change_module_array.hpp>
#include <tier4_planning_msgs/msg/path_change_module_id.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace behavior_path_planner
{
using ApprovalMsg = tier4_planning_msgs::msg::Approval;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::HADMapRoute;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using rcl_interfaces::msg::SetParametersResult;
using route_handler::RouteHandler;
using tier4_planning_msgs::msg::AvoidanceDebugFactor;
using tier4_planning_msgs::msg::AvoidanceDebugMsg;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;
using tier4_planning_msgs::msg::PathChangeModule;
using tier4_planning_msgs::msg::PathChangeModuleArray;
using tier4_planning_msgs::msg::Scenario;
using visualization_msgs::msg::MarkerArray;

template <typename T>
inline void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  const auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != parameters.cend()) {
    value = static_cast<T>(it->template get_value<T>());
  }
}

class BehaviorPathPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<HADMapRoute>::SharedPtr route_subscriber_;
  rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_subscriber_;
  rclcpp::Subscription<Odometry>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<Scenario>::SharedPtr scenario_subscriber_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr perception_subscriber_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Subscription<ApprovalMsg>::SharedPtr external_approval_subscriber_;
  rclcpp::Subscription<PathChangeModule>::SharedPtr force_approval_subscriber_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr path_publisher_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_signal_publisher_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_signal_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<PlannerData> planner_data_;
  std::shared_ptr<BehaviorTreeManager> bt_manager_;
  tier4_autoware_utils::SelfPoseListener self_pose_listener_{this};
  Scenario::SharedPtr current_scenario_{nullptr};

  std::string prev_ready_module_name_ = "NONE";
  PathChangeModule ready_module_{};
  PathChangeModuleArray running_modules_{};

  TurnSignalDecider turn_signal_decider_;

  std::mutex mutex_pd_;  // mutex for planner_data_
  std::mutex mutex_bt_;  // mutex for bt_manager_

  // setup
  void waitForData();
  std::shared_ptr<BehaviorPathPlannerParameters> common_param_ptr_;
  std::shared_ptr<LaneChangeParameters> lane_change_param_ptr;

  // parameters
  BehaviorPathPlannerParameters getCommonParam();
  BehaviorTreeManagerParam getBehaviorTreeManagerParam();
  SideShiftParameters getSideShiftParam();
  AvoidanceParameters getAvoidanceParam();
  LaneFollowingParameters getLaneFollowingParam();
  LaneChangeParameters getLaneChangeParam();
  PullOverParameters getPullOverParam();
  PullOutParameters getPullOutParam();

  // callback
  void onVelocity(const Odometry::ConstSharedPtr msg);
  void onPerception(const PredictedObjects::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onExternalApproval(const ApprovalMsg::ConstSharedPtr msg);
  void onForceApproval(const PathChangeModule::ConstSharedPtr msg);
  void onMap(const HADMapBin::ConstSharedPtr map_msg);
  void onRoute(const HADMapRoute::ConstSharedPtr route_msg);
  SetParametersResult onSetParam(const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  /**
   * @brief Modify the path points near the goal to smoothly connect the lanelet and the goal point.
   */
  PathWithLaneId modifyPathForSmoothGoalConnection(
    const PathWithLaneId & path) const;  // (TODO) move to util

  void clipPathLength(PathWithLaneId & path) const;  // (TODO) move to util

  /**
   * @brief Execute behavior tree and publish planned data.
   */
  void run();

  /**
   * @brief extract path from behavior tree output
   */
  PathWithLaneId::SharedPtr getPath(
    const BehaviorModuleOutput & bt_out, const std::shared_ptr<PlannerData> planner_data);

  /**
   * @brief skip smooth goal connection
   */
  bool skipSmoothGoalConnection(
    const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses) const;

  // debug

private:
  rclcpp::Publisher<OccupancyGrid>::SharedPtr debug_drivable_area_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_drivable_area_lanelets_publisher_;
  rclcpp::Publisher<Path>::SharedPtr debug_path_publisher_;
  rclcpp::Publisher<AvoidanceDebugMsgArray>::SharedPtr debug_avoidance_msg_array_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_publisher_;
  void publishDebugMarker(const std::vector<MarkerArray> & debug_markers);
  rclcpp::Publisher<LaneChangeDebugMsgArray>::SharedPtr debug_lane_change_msg_array_publisher_;

  /**
   * @brief check path if it is unsafe or forced
   */
  bool isForcedCandidatePath() const;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
