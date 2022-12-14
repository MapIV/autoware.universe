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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__UTIL_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__UTIL_HPP_

#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <lanelet2_core/primitives/Primitive.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace behavior_path_planner::lane_change_utils
{
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using tier4_autoware_utils::Polygon2d;

PathWithLaneId combineReferencePath(const PathWithLaneId & path1, const PathWithLaneId & path2);

bool isPathInLanelets(
  const PathWithLaneId & path, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets);

double getExpectedVelocityWhenDecelerate(
  const double & current_velocity, const double & expected_acceleration,
  const double & lane_change_prepare_duration);

std::pair<double, double> calcLaneChangingSpeedAndDistanceWhenDecelerate(
  const double velocity, const double shift_length, const double expected_acceleration,
  const double minimum_lane_change_length, const LaneChangeParameters & params);

std::optional<LaneChangePath> constructCandidatePath(
  const PathWithLaneId & prepare_segment, const PathWithLaneId & lane_changing_segment,
  const PathWithLaneId & target_lane_reference_path, const ShiftPoint & shift_point,
  const lanelet::ConstLanelets & original_lanelets, const lanelet::ConstLanelets & target_lanelets,
  const double & acceleration, const double & prepare_distance, const double & prepare_duration,
  const double & prepare_speed, const double & lane_change_distance,
  const double & lane_changing_speed, const LaneChangeParameters & params);

LaneChangePaths getLaneChangePaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets, const Pose & pose, const Twist & twist,
  const BehaviorPathPlannerParameters & common_parameter,
  const behavior_path_planner::LaneChangeParameters & parameter);

LaneChangePaths selectValidPaths(
  const LaneChangePaths & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const RouteHandler & route_handler,
  const Pose & current_pose, const Pose & goal_pose, const double minimum_lane_change_length);

bool selectSafePath(
  const LaneChangePaths & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const PredictedObjects::ConstSharedPtr dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const BehaviorPathPlannerParameters & common_parameters,
  const behavior_path_planner::LaneChangeParameters & ros_parameters,
  LaneChangePath * selected_path,
  std::unordered_map<std::string, CollisionCheckDebug> & debug_data);

bool isLaneChangePathSafe(
  const PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const PredictedObjects::ConstSharedPtr dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters, const double front_decel,
  const double rear_decel, Pose & ego_pose_before_collision,
  std::unordered_map<std::string, CollisionCheckDebug> & debug_data, const bool use_buffer = true,
  const double acceleration = 0.0);

bool hasEnoughDistance(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Pose & current_pose, const Pose & goal_pose,
  const RouteHandler & route_handler, const double minimum_lane_change_length);

ShiftPoint getLaneChangeShiftPoint(
  const PathWithLaneId & path1, const PathWithLaneId & path2,
  const lanelet::ConstLanelets & target_lanes, const PathWithLaneId & reference_path);

PathWithLaneId getReferencePathFromTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & lane_changing_start_pose, const double & prepare_distance,
  const double & lane_changing_distance, const double & forward_path_length,
  const int & num_to_preferred_lane, const double & minimum_lane_change_length);

PathWithLaneId getReferencePathFromTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & in_target_front_pose, const Pose & in_target_end_pose);

PathWithLaneId getLaneChangePathPrepareSegment(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const Pose & current_pose, const double & backward_path_length, const double & prepare_distance,
  const double & prepare_duration, const double & minimum_lane_change_velocity);

PathWithLaneId getLaneChangePathLaneChangingSegment(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanelets,
  const Pose & current_pose, const double & forward_path_length, const double & prepare_distance,
  const double & lane_change_distance, const double & minimum_lane_change_length,
  const double & lane_change_distance_buffer, const double & lane_changing_speed);
bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param);
bool isEgoDistanceNearToCenterline(
  const lanelet::ConstLanelet & closest_lanelet, const Pose & current_pose,
  const LaneChangeParameters & lane_change_param);
bool isEgoHeadingAngleLessThanThreshold(
  const lanelet::ConstLanelet & closest_lanelet, const Pose & current_pose,
  const LaneChangeParameters & lane_change_param);
void get_turn_signal_info(
  const LaneChangePath & lane_change_path, TurnSignalInfo * turn_signal_info);

double abortPointDistance(
  const double starting_velocity, const double param_accel, const double param_jerk,
  const double param_time);
std::optional<LaneChangePath> getAbortPaths(
  const std::shared_ptr<const PlannerData> & planner_data, const LaneChangePath & selected_path,
  const Pose & ego_lerp_pose_before_collision, const BehaviorPathPlannerParameters & common_param,
  const LaneChangeParameters & lane_change_param);

double getLateralShift(const LaneChangePath & path);

bool hasEnoughDistanceToLaneChangeAfterAbort(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const Pose & curent_pose, const double abort_return_dist,
  const BehaviorPathPlannerParameters & common_param,
  const LaneChangeParameters & lane_change_param);
}  // namespace behavior_path_planner::lane_change_utils

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__UTIL_HPP_
