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

#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/lane_change/util.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

std::string to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;

LaneChangeModule::LaneChangeModule(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters)
: SceneModuleInterface{name, node},
  parameters_{std::move(parameters)},
  rtc_interface_left_(&node, "lane_change_left"),
  rtc_interface_right_(&node, "lane_change_right"),
  uuid_left_{generateUUID()},
  uuid_right_{generateUUID()}
{
}

void LaneChangeModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onEntry");
  current_state_ = BT::NodeStatus::SUCCESS;
  current_lane_change_state_ = LaneChangeStates::Normal;
  updateLaneChangeStatus();
}

void LaneChangeModule::onExit()
{
  resetParameters();
  resetPathCandidate();
  current_state_ = BT::NodeStatus::SUCCESS;
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onExit");
}

bool LaneChangeModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  // Get lane change lanes
  const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_valid_path;
}

bool LaneChangeModule::isExecutionReady() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  // Get lane change lanes
  const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_safe_path;
}

BT::NodeStatus LaneChangeModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE updateState");
  if (isAbortConditionSatisfied()) {
    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      current_state_ = BT::NodeStatus::RUNNING;
      return current_state_;
    }

    if (isAbortState()) {
      current_state_ = BT::NodeStatus::RUNNING;
      return current_state_;
    }

    current_state_ = BT::NodeStatus::FAILURE;
    return current_state_;
  }

  if (hasFinishedLaneChange()) {
    lane_change_debug_msg_array_.lane_change_info.clear();
    abort_path_ = nullptr;
    current_state_ = BT::NodeStatus::SUCCESS;
    return current_state_;
  }
  current_state_ = BT::NodeStatus::RUNNING;
  return current_state_;
}

BehaviorModuleOutput LaneChangeModule::plan()
{
  resetPathCandidate();
  is_activated_ = isActivated();

  PathWithLaneId path = status_.lane_change_path.path;

  if (isAbortConditionSatisfied()) {
    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      const auto stop_point = util::insertStopPoint(0.1, &path);
    }
  }

  if (isAbortState()) {
    resetPathIfAbort();
    if (is_activated_) {
      path = abort_path_->path;
    }
  }

  generateExtendedDrivableArea(path);

  if (!isAbortState()) {
    prev_approved_path_ = path;
  }

  BehaviorModuleOutput output;

  output.path = std::make_shared<PathWithLaneId>(path);

  updateOutputTurnSignal(output);
  const auto turn_signal_info = output.turn_signal_info;
  if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
    waitApprovalLeft(turn_signal_info.signal_distance);
  } else if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
    waitApprovalRight(turn_signal_info.signal_distance);
  }

  if (!isSafe()) {
    current_state_ = BT::NodeStatus::SUCCESS;  // for breaking loop
  }

  return output;
}

void LaneChangeModule::resetPathIfAbort()
{
  if (!is_abort_approval_requested_) {
    const auto lateral_shift = lane_change_utils::getLateralShift(*abort_path_);
    if (lateral_shift > 0.0) {
      removePreviousRTCStatusRight();
      uuid_right_ = generateUUID();
    } else if (lateral_shift < 0.0) {
      removePreviousRTCStatusLeft();
      uuid_left_ = generateUUID();
    }
    is_abort_approval_requested_ = true;
    is_abort_path_approved_ = false;
    RCLCPP_ERROR(getLogger(), "[plan] uuid is reset to request abort approval.");
    return;
  }

  if (isActivated()) {
    RCLCPP_INFO(getLogger(), "[plan] isActivated() is true. set is_abort_path_approved to true.");
    is_abort_path_approved_ = true;
    clearWaitingApproval();
  } else {
    RCLCPP_INFO(getLogger(), "[plan] isActivated() is False.");
    is_abort_path_approved_ = false;
    waitApproval();
  }
}

CandidateOutput LaneChangeModule::planCandidate() const
{
  CandidateOutput output;

  LaneChangePath selected_path;
  // Get lane change lanes
  const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  [[maybe_unused]] const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);
  selected_path.path.header = planner_data_->route_handler->getRouteHeader();

  if (isAbortState()) {
    selected_path = *abort_path_;
  }

  if (selected_path.path.points.empty()) {
    return output;
  }

  const auto & start_idx = selected_path.shift_point.start_idx;
  const auto & end_idx = selected_path.shift_point.end_idx;

  output.path_candidate = selected_path.path;
  output.lateral_shift = selected_path.shifted_path.shift_length.at(end_idx) -
                         selected_path.shifted_path.shift_length.at(start_idx);
  output.distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, planner_data_->self_pose->pose.position,
    selected_path.shift_point.start.position);

  return output;
}

BehaviorModuleOutput LaneChangeModule::planWaitingApproval()
{
  BehaviorModuleOutput out;

  const auto is_within_current_lane = lane_change_utils::isEgoWithinOriginalLane(
    status_.current_lanes, getEgoPose(), planner_data_->parameters);

  if (is_within_current_lane) {
    prev_approved_path_ = getReferencePath();
  }
  out.path = std::make_shared<PathWithLaneId>(prev_approved_path_);

  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);

  if (isRequireTurnSignalWithoutApproval(prev_approved_path_)) {
    const auto direction = getLaneChangeDirection(status_.lane_change_path);
    if (direction == LaneChangeDirection::LEFT) {
      out.turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
    }
    if (direction == LaneChangeDirection::RIGHT) {
      out.turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    }
    if (direction == LaneChangeDirection::NONE) {
      out.turn_signal_info.turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
    }
  }

  updateRTCStatus(candidate);
  waitApproval();
  is_abort_path_approved_ = false;
  return out;
}

void LaneChangeModule::updateLaneChangeStatus()
{
  status_.current_lanes = util::getCurrentLanes(planner_data_);
  status_.lane_change_lanes = getLaneChangeLanes(status_.current_lanes, lane_change_lane_length_);

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(status_.lane_change_lanes, check_distance_, selected_path);

  // Update status
  status_.is_safe = found_safe_path;
  status_.lane_change_path = selected_path;
  status_.lane_follow_lane_ids = util::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = util::getIds(status_.lane_change_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

PathWithLaneId LaneChangeModule::getReferencePath() const
{
  PathWithLaneId reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto & common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  const auto current_lanes = util::getCurrentLanes(planner_data_);

  if (current_lanes.empty()) {
    return reference_path;
  }

  if (reference_path.points.empty()) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters);
  }

  double optional_lengths{0.0};
  reference_path = util::getCenterLinePath(
    *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length, common_parameters, optional_lengths);
  const int num_lane_change =
    std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));
  const double lane_change_buffer =
    util::calcLaneChangeBuffer(common_parameters, num_lane_change, optional_lengths);

  reference_path = util::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, parameters_->lane_change_prepare_duration,
    lane_change_buffer);

  reference_path.drivable_area = util::generateDrivableArea(
    reference_path, current_lanes, common_parameters.drivable_area_resolution,
    common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

lanelet::ConstLanelets LaneChangeModule::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const
{
  lanelet::ConstLanelets lane_change_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto & minimum_lane_change_length = planner_data_->parameters.minimum_lane_change_length;
  const auto & lane_change_prepare_duration = parameters_->lane_change_prepare_duration;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();

  if (current_lanes.empty()) {
    return lane_change_lanes;
  }

  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &current_lane);
  const double lane_change_prepare_length =
    std::max(current_twist.linear.x * lane_change_prepare_duration, minimum_lane_change_length);
  lanelet::ConstLanelets current_check_lanes =
    route_handler->getLaneletSequence(current_lane, current_pose, 0.0, lane_change_prepare_length);
  lanelet::ConstLanelet lane_change_lane;
  if (route_handler->getLaneChangeTarget(current_check_lanes, &lane_change_lane)) {
    lane_change_lanes = route_handler->getLaneletSequence(
      lane_change_lane, current_pose, lane_change_lane_length, lane_change_lane_length);
  } else {
    lane_change_lanes.clear();
  }

  return lane_change_lanes;
}

std::pair<bool, bool> LaneChangeModule::getSafePath(
  const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
  LaneChangePath & safe_path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & common_parameters = planner_data_->parameters;

  const auto current_lanes = util::getCurrentLanes(planner_data_);

  if (!lane_change_lanes.empty()) {
    // find candidate paths
    const auto lane_change_paths = lane_change_utils::getLaneChangePaths(
      *route_handler, current_lanes, lane_change_lanes, current_pose, current_twist,
      common_parameters, *parameters_);

    // get lanes used for detection
    lanelet::ConstLanelets check_lanes;
    if (!lane_change_paths.empty()) {
      const auto & longest_path = lane_change_paths.front();
      // we want to see check_distance [m] behind vehicle so add lane changing length
      const double check_distance_with_path = check_distance + longest_path.length.sum();
      check_lanes = route_handler->getCheckTargetLanesFromPath(
        longest_path.path, lane_change_lanes, check_distance_with_path);
    }

    // select valid path
    const LaneChangePaths valid_paths = lane_change_utils::selectValidPaths(
      lane_change_paths, current_lanes, check_lanes, *route_handler, current_pose,
      route_handler->getGoalPose(),
      common_parameters.minimum_lane_change_length +
        common_parameters.backward_length_buffer_for_end_of_lane +
        parameters_->lane_change_finish_judge_buffer);

    if (valid_paths.empty()) {
      return std::make_pair(false, false);
    }
    debug_valid_path_ = valid_paths;

    // select safe path
    const auto lanes = LaneChangeLanes{current_lanes, check_lanes};
    const bool found_safe_path = lane_change_utils::selectSafePath(
      valid_paths, *route_handler, lanes, planner_data_->dynamic_object, current_pose,
      current_twist, common_parameters, *parameters_, &safe_path, object_debug_);

    if (parameters_->publish_debug_marker) {
      setObjectDebugVisualization();
    } else {
      debug_marker_.markers.clear();
    }

    return std::make_pair(true, found_safe_path);
  }

  return std::make_pair(false, false);
}

bool LaneChangeModule::isSafe() const { return status_.is_safe; }

bool LaneChangeModule::isNearEndOfLane() const
{
  const auto & current_pose = getEgoPose();
  const double threshold = util::calcTotalLaneChangeDistanceWithBuffer(planner_data_->parameters);

  return std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

bool LaneChangeModule::isRequireTurnSignalWithoutApproval(const PathWithLaneId & path) const
{
  const auto & current_pose = getEgoPose();
  const auto ego_speed = util::l2Norm(getEgoTwist().linear);
  const double threshold = ego_speed * 3.0;
  const double distance_to_end_of_lane = motion_utils::calcSignedArcLength(
    path.points, current_pose.position, path.points.back().point.pose.position);
  // std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes));

  return (distance_to_end_of_lane < threshold) || (distance_to_end_of_lane <= 30.0);
}

bool LaneChangeModule::isCurrentSpeedLow() const
{
  const auto & current_twist = planner_data_->self_odometry->twist.twist;
  constexpr double threshold_kmph = 10.0;
  return util::l2Norm(current_twist.linear) < threshold_kmph * 1000 / 3600;
}

bool LaneChangeModule::isAbortConditionSatisfied()
{
  const auto & common_parameters = planner_data_->parameters;
  is_abort_condition_satisfied_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;

  // check cancel enable flag
  if (!parameters_->enable_cancel_lane_change) {
    return false;
  }

  if (!is_activated_) {
    return false;
  }

  // check if lane change path is still safe
  Pose ego_pose_before_collision;
  const auto is_path_safe = isApprovedPathSafe(ego_pose_before_collision);

  if (is_path_safe) {
    return false;
  }

  if (isNearEndOfLane()) {
    return false;
  }

  const auto is_within_original_lane = lane_change_utils::isEgoWithinOriginalLane(
    status_.current_lanes, getEgoPose(), common_parameters);

  if (is_within_original_lane) {
    return true;
  }

  // check abort enable flag
  if (!parameters_->enable_abort_lane_change) {
    // because ego is not within, and we disable abort,
    // so we can assume that abort condition is not satisfied.
    abort_path_ = nullptr;
    return false;
  }

  RCLCPP_WARN_STREAM_THROTTLE(
    getLogger(), *clock_, 1000,
    "DANGER!!! Path is not safe anymore, but it is too late to abort! Please be cautious");

  current_lane_change_state_ = LaneChangeStates::Abort;

  const auto found_abort_path = lane_change_utils::getAbortPaths(
    planner_data_, status_.lane_change_path, ego_pose_before_collision, common_parameters,
    *parameters_);

  abort_non_collision_pose_ = ego_pose_before_collision;

  if (found_abort_path) {
    if (!is_abort_path_approved_) {
      abort_path_ = std::make_shared<LaneChangePath>(*found_abort_path);
    }
    return true;
  }

  return false;
}

bool LaneChangeModule::isAbortState() const
{
  if (!parameters_->enable_abort_lane_change) {
    return false;
  }

  if (!abort_path_) {
    return false;
  }
  return (current_lane_change_state_ == LaneChangeStates::Abort);
}

bool LaneChangeModule::isCancelState() const
{
  return (current_lane_change_state_ == LaneChangeStates::Cancel);
}

bool LaneChangeModule::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  const double travel_distance = arclength_current.length - status_.start_distance;
  const double finish_distance =
    status_.lane_change_path.length.sum() + parameters_->lane_change_finish_judge_buffer;
  return travel_distance > finish_distance;
}

void LaneChangeModule::setObjectDebugVisualization() const
{
  using marker_utils::lane_change_markers::showAllValidLaneChangePath;
  using marker_utils::lane_change_markers::showLerpedPose;
  using marker_utils::lane_change_markers::showObjectInfo;
  using marker_utils::lane_change_markers::showPolygon;
  using marker_utils::lane_change_markers::showPolygonPose;

  debug_marker_.markers.clear();
  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  add(showObjectInfo(object_debug_, "object_debug_info"));
  add(showLerpedPose(object_debug_, "lerp_pose_before_true"));
  add(showPolygonPose(object_debug_, "expected_pose"));
  add(showPolygon(object_debug_, "lerped_polygon"));
  add(showAllValidLaneChangePath(debug_valid_path_, "lane_change_valid_paths"));
}

std::shared_ptr<LaneChangeDebugMsgArray> LaneChangeModule::get_debug_msg_array() const
{
  LaneChangeDebugMsgArray debug_msg_array;
  debug_msg_array.lane_change_info.reserve(object_debug_.size());
  for (const auto & [uuid, debug_data] : object_debug_) {
    LaneChangeDebugMsg debug_msg;
    debug_msg.object_id = uuid;
    debug_msg.allow_lane_change = debug_data.allow_lane_change;
    debug_msg.is_front = debug_data.is_front;
    debug_msg.relative_distance = debug_data.relative_to_ego;
    debug_msg.failed_reason = debug_data.failed_reason;
    debug_msg.velocity = util::l2Norm(debug_data.object_twist.linear);
    debug_msg_array.lane_change_info.push_back(debug_msg);
  }
  lane_change_debug_msg_array_ = debug_msg_array;

  lane_change_debug_msg_array_.header.stamp = clock_->now();
  return std::make_shared<LaneChangeDebugMsgArray>(lane_change_debug_msg_array_);
}
Pose LaneChangeModule::getEgoPose() const { return planner_data_->self_pose->pose; }
Twist LaneChangeModule::getEgoTwist() const { return planner_data_->self_odometry->twist.twist; }
std_msgs::msg::Header LaneChangeModule::getRouteHeader() const
{
  return planner_data_->route_handler->getRouteHeader();
}
void LaneChangeModule::generateExtendedDrivableArea(PathWithLaneId & path)
{
  const auto & common_parameters = planner_data_->parameters;
  lanelet::ConstLanelets lanes;
  lanes.reserve(status_.current_lanes.size() + status_.lane_change_lanes.size());
  lanes.insert(lanes.end(), status_.current_lanes.begin(), status_.current_lanes.end());
  lanes.insert(lanes.end(), status_.lane_change_lanes.begin(), status_.lane_change_lanes.end());

  const double & resolution = common_parameters.drivable_area_resolution;
  path.drivable_area = util::generateDrivableArea(
    path, lanes, resolution, common_parameters.vehicle_length, planner_data_);
}

bool LaneChangeModule::isApprovedPathSafe(Pose & ego_pose_before_collision) const
{
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameters = planner_data_->parameters;
  const auto & path = status_.lane_change_path;

  std::unordered_map<std::string, CollisionCheckDebug> debug_data;

  const auto lanes = LaneChangeLanes{
    status_.lane_change_path.reference_lanelets, status_.lane_change_path.target_lanelets};
  return lane_change_utils::isLaneChangePathSafe(
    path, *planner_data_->route_handler, lanes, dynamic_objects, current_pose, current_twist,
    common_parameters, *parameters_, common_parameters.expected_front_deceleration_for_abort,
    common_parameters.expected_rear_deceleration_for_abort, path.length, path.duration,
    ego_pose_before_collision, debug_data, false, status_.lane_change_path.acceleration);
}

void LaneChangeModule::updateOutputTurnSignal(BehaviorModuleOutput & output)
{
  const auto turn_signal_info = util::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.shift_point, getEgoPose(), getEgoTwist().linear.x,
    planner_data_->parameters);
  output.turn_signal_info.turn_signal.command = turn_signal_info.first.command;
  output.turn_signal_info.signal_distance = turn_signal_info.second;
}

void LaneChangeModule::resetParameters()
{
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;
  abort_path_ = nullptr;

  clearWaitingApproval();
  removeRTCStatus();
  object_debug_.clear();
  debug_marker_.markers.clear();
}

[[nodiscard]] LaneChangeDirection LaneChangeModule::getLaneChangeDirection(
  const LaneChangePath & path)
{
  const auto lateral_shift = -lane_change_utils::getLateralShift(path);
  constexpr double epsilon = 1.0e-5;
  if (lateral_shift < -epsilon) {
    return LaneChangeDirection::LEFT;
  }
  if (lateral_shift > epsilon) {
    return LaneChangeDirection::RIGHT;
  }
  return LaneChangeDirection::NONE;
}

void LaneChangeModule::accept_visitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visit_lane_change_module(this);
  }
}

void LaneChangeVisitor::visit_lane_change_module(const LaneChangeModule * module) const
{
  lane_change_visitor_ = module->get_debug_msg_array();
}

void LaneChangeVisitor::visit_avoidance_module(
  [[maybe_unused]] const AvoidanceModule * module) const
{
}
}  // namespace behavior_path_planner
