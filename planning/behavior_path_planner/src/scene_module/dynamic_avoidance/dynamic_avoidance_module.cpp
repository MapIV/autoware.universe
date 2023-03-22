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

#include "behavior_path_planner/marker_util/avoidance/debug.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/util/avoidance/util.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tier4_planning_msgs/msg/avoidance_debug_factor.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

// set as macro so that calling function name will be printed.
// debug print is heavy. turn on only when debugging.
#define DEBUG_PRINT(...) \
  RCLCPP_DEBUG_EXPRESSION(getLogger(), parameters_->print_debug_info, __VA_ARGS__)
#define printShiftLines(p, msg) DEBUG_PRINT("[%s] %s", msg, toStrInfo(p).c_str())

namespace behavior_path_planner
{
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcInterpolatedPose;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcLongitudinalDeviation;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;
using tier4_planning_msgs::msg::DynamicAvoidanceDebugFactor;

DynamicAvoidanceModule::DynamicAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  std::shared_ptr<DynamicAvoidanceParameters> parameters,
  std::shared_ptr<RTCInterface> & rtc_interface_left,
  std::shared_ptr<RTCInterface> & rtc_interface_right)
: SceneModuleInterface{name, node},
  parameters_{std::move(parameters)},
  rtc_interface_left_{rtc_interface_left},
  rtc_interface_right_{rtc_interface_right},
  uuid_left_{generateUUID()},
  uuid_right_{generateUUID()}
{
  using std::placeholders::_1;
  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(&node, "avoidance");
}

bool DynamicAvoidanceModule::isExecutionRequested() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionRequested");

  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  const auto avoid_data = calcDynamicAvoidancePlanningData(debug_data_);
  return !avoid_data.target_objects.empty();
}

bool DynamicAvoidanceModule::isExecutionReady() const
{
  DEBUG_PRINT("AVOIDANCE isExecutionReady");

  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  return true;
}

ModuleStatus DynamicAvoidanceModule::updateState()
{
  const bool has_avoidance_target = !avoidance_data_.target_objects.empty();

  if (!has_avoidance_target) {
    current_state_ = ModuleStatus::SUCCESS;
  } else {
    current_state_ = ModuleStatus::RUNNING;
  }

  DEBUG_PRINT("has_avoidance_target = %d", has_avoidance_target);

  return current_state_;
}

DynamicAvoidancePlanningData DynamicAvoidanceModule::calcDynamicAvoidancePlanningData(
  DebugData & debug) const
{
  DynamicAvoidancePlanningData data;
  return data;
}

// TODO(murooka) judge when and which way to extend drivable area. current implementation is keep
// extending during avoidance module
// TODO(murooka) freespace during turning in intersection where there is no neighbor lanes
// NOTE: Assume that there is no situation where there is an object in the middle lane of more than
// two lanes since which way to avoid is not obvious
void DynamicAvoidanceModule::generateExtendedDrivableArea(PathWithLaneId & path) const
{
  const auto & p = planner_data_->parameters;
  generateDrivableArea(
    path, drivable_lanes, p.vehicle_length, planner_data_, avoidance_data_.target_objects,
    parameters_->enable_bound_clipping, parameters_->disable_path_update,
    parameters_->object_envelope_buffer);
}

BehaviorModuleOutput DynamicAvoidanceModule::plan()
{
  const auto & data = avoidance_data_;

  resetPathCandidate();
  resetPathReference();

  /**
   * Has new shift point?
   *   Yes -> Is it approved?
   *       Yes -> add the shift point.
   *       No  -> set approval_handler to WAIT_APPROVAL state.
   *   No -> waiting approval?
   *       Yes -> clear WAIT_APPROVAL state.
   *       No  -> do nothing.
   */
  if (!data.safe_new_sl.empty()) {
    debug_data_.new_shift_lines = data.safe_new_sl;
    DEBUG_PRINT("new_shift_lines size = %lu", data.safe_new_sl.size());
    printShiftLines(data.safe_new_sl, "new_shift_lines");

    const auto sl = getNonStraightShiftLine(data.safe_new_sl);
    if (getRelativeLengthFromPath(sl) > 0.0) {
      removePreviousRTCStatusRight();
    } else if (getRelativeLengthFromPath(sl) < 0.0) {
      removePreviousRTCStatusLeft();
    } else {
      RCLCPP_WARN_STREAM(getLogger(), "Direction is UNKNOWN");
    }
    if (!parameters_->disable_path_update) {
      addShiftLineIfApproved(data.safe_new_sl);
    }
  } else if (isWaitingApproval()) {
    clearWaitingApproval();
    removeCandidateRTCStatus();
  }

  // generate path with shift points that have been inserted.
  auto avoidance_path = generateDynamicAvoidancePath(path_shifter_);
  debug_data_.output_shift = avoidance_path.shift_length;

  // modify max speed to prevent acceleration in avoidance maneuver.
  modifyPathVelocityToPreventAccelerationOnDynamicAvoidance(avoidance_path);

  // post processing
  {
    postProcess(path_shifter_);  // remove old shift points
    prev_output_ = avoidance_path;
    prev_linear_shift_path_ = toShiftedPath(avoidance_data_.reference_path);
    path_shifter_.generate(&prev_linear_shift_path_, true, SHIFT_TYPE::LINEAR);
    prev_reference_ = avoidance_data_.reference_path;
  }

  BehaviorModuleOutput output;
  output.turn_signal_info = calcTurnSignalInfo(avoidance_path);
  // sparse resampling for computational cost
  {
    avoidance_path.path =
      util::resamplePathWithSpline(avoidance_path.path, parameters_->resample_interval_for_output);
  }

  avoidance_data_.state = updateEgoState(data);
  if (!parameters_->disable_path_update) {
    updateEgoBehavior(data, avoidance_path);
  }

  if (parameters_->publish_debug_marker) {
    setDebugData(avoidance_data_, path_shifter_, debug_data_);
  } else {
    debug_marker_.markers.clear();
  }

  output.path = std::make_shared<PathWithLaneId>(avoidance_path.path);
  output.reference_path = getPreviousModuleOutput().reference_path;

  const size_t ego_idx = planner_data_->findEgoIndex(output.path->points);
  util::clipPathLength(*output.path, ego_idx, planner_data_->parameters);

  // Drivable area generation.
  generateExtendedDrivableArea(*output.path);

  DEBUG_PRINT("exit plan(): set prev output (back().lat = %f)", prev_output_.shift_length.back());

  updateRegisteredRTCStatus(avoidance_path.path);

  return output;
}

CandidateOutput DynamicAvoidanceModule::planCandidate() const
{
  const auto & data = avoidance_data_;

  CandidateOutput output;

  auto shifted_path = data.candidate_path;

  if (!data.safe_new_sl.empty()) {  // clip from shift start index for visualize
    clipByMinStartIdx(data.safe_new_sl, shifted_path.path);

    const auto sl = getNonStraightShiftLine(data.safe_new_sl);
    const auto sl_front = data.safe_new_sl.front();
    const auto sl_back = data.safe_new_sl.back();

    output.lateral_shift = getRelativeLengthFromPath(sl);
    output.start_distance_to_path_change = sl_front.start_longitudinal;
    output.finish_distance_to_path_change = sl_back.end_longitudinal;

    const uint16_t steering_factor_direction = std::invoke([&output]() {
      if (output.lateral_shift > 0.0) {
        return SteeringFactor::LEFT;
      }
      return SteeringFactor::RIGHT;
    });
    steering_factor_interface_ptr_->updateSteeringFactor(
      {sl_front.start, sl_back.end},
      {output.start_distance_to_path_change, output.finish_distance_to_path_change},
      SteeringFactor::AVOIDANCE_PATH_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING,
      "");
  }

  const size_t ego_idx = planner_data_->findEgoIndex(shifted_path.path.points);
  util::clipPathLength(shifted_path.path, ego_idx, planner_data_->parameters);

  output.path_candidate = shifted_path.path;

  return output;
}

BehaviorModuleOutput DynamicAvoidanceModule::planWaitingApproval()
{
  // we can execute the plan() since it handles the approval appropriately.
  BehaviorModuleOutput out = plan();
  if (path_shifter_.getShiftLines().empty()) {
    out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  }
  const auto candidate = planCandidate();
  constexpr double threshold_to_update_status = -1.0e-03;
  if (candidate.start_distance_to_path_change > threshold_to_update_status) {
    updateCandidateRTCStatus(candidate);
    waitApproval();
  } else {
    clearWaitingApproval();
    removeCandidateRTCStatus();
  }
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);
  path_reference_ = getPreviousModuleOutput().reference_path;
  return out;
}

void DynamicAvoidanceModule::addShiftLineIfApproved(const AvoidLineArray & shift_lines)
{
  if (isActivated()) {
    DEBUG_PRINT("We want to add this shift point, and approved. ADD SHIFT POINT!");
    const size_t prev_size = path_shifter_.getShiftLinesSize();
    addNewShiftLines(path_shifter_, shift_lines);

    current_raw_shift_lines_ = avoidance_data_.unapproved_raw_sl;

    // register original points for consistency
    registerRawShiftLines(shift_lines);

    const auto sl = getNonStraightShiftLine(shift_lines);
    const auto sl_front = shift_lines.front();
    const auto sl_back = shift_lines.back();

    if (getRelativeLengthFromPath(sl) > 0.0) {
      left_shift_array_.push_back({uuid_left_, sl_front.start, sl_back.end});
    } else if (getRelativeLengthFromPath(sl) < 0.0) {
      right_shift_array_.push_back({uuid_right_, sl_front.start, sl_back.end});
    }

    uuid_left_ = generateUUID();
    uuid_right_ = generateUUID();
    candidate_uuid_ = generateUUID();

    lockNewModuleLaunch();

    DEBUG_PRINT("shift_line size: %lu -> %lu", prev_size, path_shifter_.getShiftLinesSize());
  } else {
    DEBUG_PRINT("We want to add this shift point, but NOT approved. waiting...");
    waitApproval();
  }
}

/**
 * set new shift points. remove old shift points if it has a conflict.
 */
void DynamicAvoidanceModule::addNewShiftLines(
  PathShifter & path_shifter, const AvoidLineArray & new_shift_lines) const
{
  ShiftLineArray future = toShiftLineArray(new_shift_lines);

  size_t min_start_idx = std::numeric_limits<size_t>::max();
  for (const auto & sl : new_shift_lines) {
    min_start_idx = std::min(min_start_idx, sl.start_idx);
  }

  const auto current_shift_lines = path_shifter.getShiftLines();

  DEBUG_PRINT("min_start_idx = %lu", min_start_idx);

  // Remove shift points that starts later than the new_shift_line from path_shifter.
  //
  // Why? Because shifter sorts by start position and applies shift points, so if there is a
  // shift point that starts after the one you are going to put in, new one will be affected
  // by the old one.
  //
  // Is it ok? This is a situation where the vehicle was originally going to avoid at the farther
  // point, but decided to avoid it at a closer point. In this case, it is reasonable to cancel the
  // farther avoidance.
  for (const auto & sl : current_shift_lines) {
    if (sl.start_idx >= min_start_idx) {
      DEBUG_PRINT(
        "sl.start_idx = %lu, this sl starts after new proposal. remove this one.", sl.start_idx);
    } else {
      DEBUG_PRINT("sl.start_idx = %lu, no conflict. keep this one.", sl.start_idx);
      future.push_back(sl);
    }
  }

  path_shifter.setShiftLines(future);
}

AvoidLineArray DynamicAvoidanceModule::findNewShiftLine(
  const AvoidLineArray & candidates, const PathShifter & shifter) const
{
  (void)shifter;

  if (candidates.empty()) {
    DEBUG_PRINT("shift candidates is empty. return None.");
    return {};
  }

  printShiftLines(candidates, "findNewShiftLine: candidates");

  // Retrieve the subsequent linear shift point from the given index point.
  const auto getShiftLineWithSubsequentStraight = [this, &candidates](size_t i) {
    AvoidLineArray subsequent{candidates.at(i)};
    for (size_t j = i + 1; j < candidates.size(); ++j) {
      const auto next_shift = candidates.at(j);
      if (std::abs(next_shift.getRelativeLength()) < 1.0e-2) {
        subsequent.push_back(next_shift);
        DEBUG_PRINT("j = %lu, relative shift is zero. add together.", j);
      } else {
        DEBUG_PRINT("j = %lu, relative shift is not zero = %f.", j, next_shift.getRelativeLength());
        break;
      }
    }
    return subsequent;
  };

  const auto calcJerk = [this](const auto & al) {
    return path_shifter_.calcJerkFromLatLonDistance(
      al.getRelativeLength(), al.getRelativeLongitudinal(), getSharpDynamicAvoidanceEgoSpeed());
  };

  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates.at(i);
    std::stringstream ss;
    ss << "i = " << i << ", id = " << candidate.id;
    const auto pfx = ss.str().c_str();

    if (prev_reference_.points.size() != prev_linear_shift_path_.path.points.size()) {
      throw std::logic_error("prev_reference_ and prev_linear_shift_path_ must have same size.");
    }

    // new shift points must exist in front of Ego
    // this value should be larger than -eps consider path shifter calculation error.
    const double eps = 0.01;
    if (candidate.start_longitudinal < -eps) {
      continue;
    }

    // TODO(Horibe): this code prohibits the changes on ego pose. Think later.
    // if (candidate.start_idx < avoidance_data_.ego_closest_path_index) {
    //   DEBUG_PRINT("%s, start_idx is behind ego. skip.", pfx);
    //   continue;
    // }

    const auto current_shift = prev_linear_shift_path_.shift_length.at(
      findNearestIndex(prev_reference_.points, candidate.end.position));

    // TODO(Horibe) test fails with this print. why?
    // DEBUG_PRINT("%s, shift current: %f, candidate: %f", pfx, current_shift,
    // candidate.end_shift_length);

    const auto new_point_threshold = parameters_->avoidance_execution_lateral_threshold;
    if (std::abs(candidate.end_shift_length - current_shift) > new_point_threshold) {
      if (calcJerk(candidate) > parameters_->max_lateral_jerk) {
        DEBUG_PRINT(
          "%s, Failed to find new shift: jerk limit over (%f).", pfx, calcJerk(candidate));
        break;
      }

      DEBUG_PRINT(
        "%s, New shift point is found!!! shift change: %f -> %f", pfx, current_shift,
        candidate.end_shift_length);
      return getShiftLineWithSubsequentStraight(i);
    }
  }

  DEBUG_PRINT("No new shift point exists.");
  return {};
}

Pose DynamicAvoidanceModule::getUnshiftedEgoPose(const ShiftedPath & prev_path) const
{
  const auto ego_pose = getEgoPose();

  if (prev_path.path.points.empty()) {
    return ego_pose;
  }

  // un-shifted fot current ideal pose
  const auto closest = findNearestIndex(prev_path.path.points, ego_pose.position);

  // NOTE: Considering avoidance by motion, we set unshifted_pose as previous path instead of
  // ego_pose.
  Pose unshifted_pose = motion_utils::calcInterpolatedPoint(prev_path.path, ego_pose).point.pose;

  util::shiftPose(&unshifted_pose, -prev_path.shift_length.at(closest));
  unshifted_pose.orientation = ego_pose.orientation;

  return unshifted_pose;
}

ShiftedPath DynamicAvoidanceModule::generateDynamicAvoidancePath(PathShifter & path_shifter) const
{
  DEBUG_PRINT("path_shifter: base shift = %f", getCurrentBaseShift());
  printShiftLines(path_shifter.getShiftLines(), "path_shifter shift points");

  ShiftedPath shifted_path;
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR(getLogger(), "failed to generate shifted path.");
    return prev_output_;
  }

  return shifted_path;
}

void DynamicAvoidanceModule::updateData()
{
  // for the first time
  if (prev_output_.path.points.empty()) {
    prev_output_.path = *getPreviousModuleOutput().path;
    prev_output_.shift_length = std::vector<double>(prev_output_.path.points.size(), 0.0);
  }
  if (prev_linear_shift_path_.path.points.empty()) {
    prev_linear_shift_path_.path = *getPreviousModuleOutput().path;
    prev_linear_shift_path_.shift_length =
      std::vector<double>(prev_linear_shift_path_.path.points.size(), 0.0);
  }
  if (prev_reference_.points.empty()) {
    prev_reference_ = *getPreviousModuleOutput().path;
  }

  debug_data_ = DebugData();
  avoidance_data_ = calcDynamicAvoidancePlanningData(debug_data_);

  // TODO(Horibe): this is not tested yet, disable now.
  updateRegisteredObject(avoidance_data_.target_objects);
  compensateDetectionLost(avoidance_data_.target_objects, avoidance_data_.other_objects);

  std::sort(
    avoidance_data_.target_objects.begin(), avoidance_data_.target_objects.end(),
    [](auto a, auto b) { return a.longitudinal < b.longitudinal; });

  path_shifter_.setPath(avoidance_data_.reference_path);

  // update registered shift point for new reference path & remove past objects
  updateRegisteredRawShiftLines();

  fillShiftLine(avoidance_data_, debug_data_);
}

/*
 * updateRegisteredObject
 *
 * Same object is observed this time -> update registered object with the new one.
 * Not observed -> increment the lost_count. if it exceeds the threshold, remove it.
 * How to check if it is same object?
 *   - it has same ID
 *   - it has different id, but sn object is found around similar position
 */
void DynamicAvoidanceModule::updateRegisteredObject(const ObjectDataArray & now_objects)
{
  const auto updateIfDetectedNow = [&now_objects, this](auto & registered_object) {
    const auto & n = now_objects;
    const auto r_id = registered_object.object.object_id;
    const auto same_id_obj = std::find_if(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });

    // same id object is detected. update registered.
    if (same_id_obj != n.end()) {
      registered_object = *same_id_obj;
      return true;
    }

    constexpr auto POS_THR = 1.5;
    const auto r_pos = registered_object.object.kinematics.initial_pose_with_covariance.pose;
    const auto similar_pos_obj = std::find_if(n.begin(), n.end(), [&](const auto & o) {
      return calcDistance2d(r_pos, o.object.kinematics.initial_pose_with_covariance.pose) < POS_THR;
    });

    // same id object is not detected, but object is found around registered. update registered.
    if (similar_pos_obj != n.end()) {
      registered_object = *similar_pos_obj;
      return true;
    }

    // Same ID nor similar position object does not found.
    return false;
  };

  // -- check registered_objects, remove if lost_count exceeds limit. --
  for (int i = static_cast<int>(registered_objects_.size()) - 1; i >= 0; --i) {
    auto & r = registered_objects_.at(i);
    const std::string s = getUuidStr(r);

    // registered object is not detected this time. lost count up.
    if (!updateIfDetectedNow(r)) {
      r.lost_time = (clock_->now() - r.last_seen).seconds();
    } else {
      r.last_seen = clock_->now();
      r.lost_time = 0.0;
    }

    // lost count exceeds threshold. remove object from register.
    if (r.lost_time > parameters_->object_last_seen_threshold) {
      registered_objects_.erase(registered_objects_.begin() + i);
    }
  }

  const auto isAlreadyRegistered = [this](const auto & n_id) {
    const auto & r = registered_objects_;
    return std::any_of(
      r.begin(), r.end(), [&n_id](const auto & o) { return o.object.object_id == n_id; });
  };

  // -- check now_objects, add it if it has new object id --
  for (const auto & now_obj : now_objects) {
    if (!isAlreadyRegistered(now_obj.object.object_id)) {
      registered_objects_.push_back(now_obj);
    }
  }
}

/*
 * CompensateDetectionLost
 *
 * add registered object if the now_objects does not contain the same object_id.
 *
 */
void DynamicAvoidanceModule::compensateDetectionLost(
  ObjectDataArray & now_objects, ObjectDataArray & other_objects) const
{
  const auto old_size = now_objects.size();  // for debug

  const auto isDetectedNow = [&](const auto & r_id) {
    const auto & n = now_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  const auto isIgnoreObject = [&](const auto & r_id) {
    const auto & n = other_objects;
    return std::any_of(
      n.begin(), n.end(), [&r_id](const auto & o) { return o.object.object_id == r_id; });
  };

  for (const auto & registered : registered_objects_) {
    if (
      !isDetectedNow(registered.object.object_id) && !isIgnoreObject(registered.object.object_id)) {
      now_objects.push_back(registered);
    }
  }
  DEBUG_PRINT("object size: %lu -> %lu", old_size, now_objects.size());
}

void DynamicAvoidanceModule::onEntry()
{
  DEBUG_PRINT("AVOIDANCE onEntry. wait approval!");
  initVariables();
  current_state_ = ModuleStatus::IDLE;
}

void DynamicAvoidanceModule::onExit()
{
  DEBUG_PRINT("AVOIDANCE onExit");
  initVariables();
  current_state_ = ModuleStatus::SUCCESS;
  clearWaitingApproval();
  removeRTCStatus();
  unlockNewModuleLaunch();
  steering_factor_interface_ptr_->clearSteeringFactors();
}

void DynamicAvoidanceModule::initVariables()
{
  prev_output_ = ShiftedPath();
  prev_linear_shift_path_ = ShiftedPath();
  prev_reference_ = PathWithLaneId();
  path_shifter_ = PathShifter{};
  left_shift_array_.clear();
  right_shift_array_.clear();

  debug_data_ = DebugData();
  debug_marker_.markers.clear();
  resetPathCandidate();
  resetPathReference();
  registered_raw_shift_lines_ = {};
  current_raw_shift_lines_ = {};
  original_unique_id = 0;
  is_avoidance_maneuver_starts = false;
}

bool DynamicAvoidanceModule::isTargetObjectType(const PredictedObject & object) const
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  const auto t = util::getHighestProbLabel(object.classification);
  const auto is_object_type =
    ((t == ObjectClassification::CAR && parameters_->avoid_car) ||
     (t == ObjectClassification::TRUCK && parameters_->avoid_truck) ||
     (t == ObjectClassification::BUS && parameters_->avoid_bus) ||
     (t == ObjectClassification::TRAILER && parameters_->avoid_trailer) ||
     (t == ObjectClassification::UNKNOWN && parameters_->avoid_unknown) ||
     (t == ObjectClassification::BICYCLE && parameters_->avoid_bicycle) ||
     (t == ObjectClassification::MOTORCYCLE && parameters_->avoid_motorcycle) ||
     (t == ObjectClassification::PEDESTRIAN && parameters_->avoid_pedestrian));
  return is_object_type;
}

TurnSignalInfo DynamicAvoidanceModule::calcTurnSignalInfo(const ShiftedPath & path) const
{
  const auto shift_lines = path_shifter_.getShiftLines();
  if (shift_lines.empty()) {
    return {};
  }

  const auto front_shift_line = shift_lines.front();
  const size_t start_idx = front_shift_line.start_idx;
  const size_t end_idx = front_shift_line.end_idx;

  const auto current_shift_length = getCurrentShift();
  const double start_shift_length = path.shift_length.at(start_idx);
  const double end_shift_length = path.shift_length.at(end_idx);
  const double segment_shift_length = end_shift_length - start_shift_length;

  const double turn_signal_shift_length_threshold =
    planner_data_->parameters.turn_signal_shift_length_threshold;
  const double turn_signal_search_time = planner_data_->parameters.turn_signal_search_time;
  const double turn_signal_minimum_search_distance =
    planner_data_->parameters.turn_signal_minimum_search_distance;

  // If shift length is shorter than the threshold, it does not need to turn on blinkers
  if (std::fabs(segment_shift_length) < turn_signal_shift_length_threshold) {
    return {};
  }

  // If the vehicle does not shift anymore, we turn off the blinker
  if (std::fabs(end_shift_length - current_shift_length) < 0.1) {
    return {};
  }

  // compute blinker start idx and end idx
  const size_t blinker_start_idx = [&]() {
    for (size_t idx = start_idx; idx <= end_idx; ++idx) {
      const double current_shift_length = path.shift_length.at(idx);
      if (current_shift_length > 0.1) {
        return idx;
      }
    }
    return start_idx;
  }();
  const size_t blinker_end_idx = end_idx;

  const auto blinker_start_pose = path.path.points.at(blinker_start_idx).point.pose;
  const auto blinker_end_pose = path.path.points.at(blinker_end_idx).point.pose;

  const double ego_vehicle_offset =
    planner_data_->parameters.vehicle_info.max_longitudinal_offset_m;
  const auto signal_prepare_distance =
    std::max(getEgoSpeed() * turn_signal_search_time, turn_signal_minimum_search_distance);
  const auto ego_front_to_shift_start =
    calcSignedArcLength(path.path.points, getEgoPosition(), blinker_start_pose.position) -
    ego_vehicle_offset;

  if (signal_prepare_distance < ego_front_to_shift_start) {
    return {};
  }

  bool turn_signal_on_swerving = planner_data_->parameters.turn_signal_on_swerving;

  TurnSignalInfo turn_signal_info{};
  if (turn_signal_on_swerving) {
    if (segment_shift_length > 0.0) {
      turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
    } else {
      turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    }
  } else {
    turn_signal_info.turn_signal.command = TurnIndicatorsCommand::DISABLE;
  }

  if (ego_front_to_shift_start > 0.0) {
    turn_signal_info.desired_start_point = planner_data_->self_odometry->pose.pose;
  } else {
    turn_signal_info.desired_start_point = blinker_start_pose;
  }
  turn_signal_info.desired_end_point = blinker_end_pose;
  turn_signal_info.required_start_point = blinker_start_pose;
  turn_signal_info.required_end_point = blinker_end_pose;

  return turn_signal_info;
}

void DynamicAvoidanceModule::setDebugData(
  const DynamicAvoidancePlanningData & data, const PathShifter & shifter,
  const DebugData & debug) const
{
  using marker_utils::createLaneletsAreaMarkerArray;
  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createShiftLengthMarkerArray;
  using marker_utils::createShiftLineMarkerArray;
  using marker_utils::avoidance_marker::createAvoidableTargetObjectsMarkerArray;
  using marker_utils::avoidance_marker::createAvoidLineMarkerArray;
  using marker_utils::avoidance_marker::createEgoStatusMarkerArray;
  using marker_utils::avoidance_marker::createOtherObjectsMarkerArray;
  using marker_utils::avoidance_marker::createOverhangFurthestLineStringMarkerArray;
  using marker_utils::avoidance_marker::createPredictedVehiclePositions;
  using marker_utils::avoidance_marker::createSafetyCheckMarkerArray;
  using marker_utils::avoidance_marker::createUnavoidableObjectsMarkerArray;
  using marker_utils::avoidance_marker::createUnavoidableTargetObjectsMarkerArray;
  using marker_utils::avoidance_marker::createUnsafeObjectsMarkerArray;
  using marker_utils::avoidance_marker::makeOverhangToRoadShoulderMarkerArray;
  using motion_utils::createDeadLineVirtualWallMarker;
  using motion_utils::createSlowDownVirtualWallMarker;
  using motion_utils::createStopVirtualWallMarker;
  using tier4_autoware_utils::appendMarkerArray;
  using tier4_autoware_utils::calcOffsetPose;

  debug_marker_.markers.clear();
  const auto & base_link2front = planner_data_->parameters.base_link2front;
  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  const auto addAvoidLine =
    [&](const AvoidLineArray & al_arr, const auto & ns, auto r, auto g, auto b, double w = 0.1) {
      add(createAvoidLineMarkerArray(al_arr, ns, r, g, b, w));
    };

  const auto addShiftLine =
    [&](const ShiftLineArray & sl_arr, const auto & ns, auto r, auto g, auto b, double w = 0.1) {
      add(createShiftLineMarkerArray(sl_arr, shifter.getBaseOffset(), ns, r, g, b, w));
    };

  add(createEgoStatusMarkerArray(data, getEgoPose(), "ego_status"));
  add(createPredictedVehiclePositions(
    debug.path_with_planned_velocity, "predicted_vehicle_positions"));

  const auto & path = data.reference_path;
  add(createPathMarkerArray(debug.center_line, "centerline", 0, 0.0, 0.5, 0.9));
  add(createPathMarkerArray(path, "centerline_resampled", 0, 0.0, 0.9, 0.5));
  add(createPathMarkerArray(prev_linear_shift_path_.path, "prev_linear_shift", 0, 0.5, 0.4, 0.6));
  add(createPoseMarkerArray(data.reference_pose, "reference_pose", 0, 0.9, 0.3, 0.3));

  if (debug.stop_pose) {
    const auto p_front = calcOffsetPose(debug.stop_pose.get(), base_link2front, 0.0, 0.0);
    appendMarkerArray(
      createStopVirtualWallMarker(p_front, "avoidance stop", current_time, 0L), &debug_marker_);
  }

  if (debug.slow_pose) {
    const auto p_front = calcOffsetPose(debug.slow_pose.get(), base_link2front, 0.0, 0.0);
    appendMarkerArray(
      createSlowDownVirtualWallMarker(p_front, "avoidance slow", current_time, 0L), &debug_marker_);
  }

  if (debug.feasible_bound) {
    const auto p_front = calcOffsetPose(debug.feasible_bound.get(), base_link2front, 0.0, 0.0);
    appendMarkerArray(
      createDeadLineVirtualWallMarker(p_front, "feasible bound", current_time, 0L), &debug_marker_);
  }

  add(createSafetyCheckMarkerArray(data.state, getEgoPose(), debug));

  std::vector<ObjectData> avoidable_target_objects;
  std::vector<ObjectData> unavoidable_target_objects;
  for (const auto & object : data.target_objects) {
    if (object.is_avoidable) {
      avoidable_target_objects.push_back(object);
    } else {
      unavoidable_target_objects.push_back(object);
    }
  }

  add(createLaneletsAreaMarkerArray(*debug.current_lanelets, "current_lanelet", 0.0, 1.0, 0.0));
  add(createLaneletsAreaMarkerArray(*debug.expanded_lanelets, "expanded_lanelet", 0.8, 0.8, 0.0));
  add(
    createAvoidableTargetObjectsMarkerArray(avoidable_target_objects, "avoidable_target_objects"));
  add(createUnavoidableTargetObjectsMarkerArray(
    unavoidable_target_objects, "unavoidable_target_objects"));
  add(createOtherObjectsMarkerArray(data.other_objects, "other_objects"));
  add(makeOverhangToRoadShoulderMarkerArray(data.target_objects, "overhang"));
  add(createOverhangFurthestLineStringMarkerArray(
    *debug.farthest_linestring_from_overhang, "farthest_linestring_from_overhang", 1.0, 0.0, 1.0));

  add(createUnavoidableObjectsMarkerArray(debug.unavoidable_objects, "unavoidable_objects"));
  add(createUnsafeObjectsMarkerArray(debug.unsafe_objects, "unsafe_objects"));

  // parent object info
  addAvoidLine(debug.registered_raw_shift, "p_registered_shift", 0.8, 0.8, 0.0);
  addAvoidLine(debug.current_raw_shift, "p_current_raw_shift", 0.5, 0.2, 0.2);
  addAvoidLine(debug.extra_return_shift, "p_extra_return_shift", 0.0, 0.5, 0.8);

  // merged shift
  const auto & linear_shift = prev_linear_shift_path_.shift_length;
  add(createShiftLengthMarkerArray(debug.pos_shift, path, "m_pos_shift_line", 0, 0.7, 0.5));
  add(createShiftLengthMarkerArray(debug.neg_shift, path, "m_neg_shift_line", 0, 0.5, 0.7));
  add(createShiftLengthMarkerArray(debug.total_shift, path, "m_total_shift_line", 0.99, 0.4, 0.2));
  add(createShiftLengthMarkerArray(debug.output_shift, path, "m_output_shift_line", 0.8, 0.8, 0.2));
  add(createShiftLengthMarkerArray(linear_shift, path, "m_output_linear_line", 0.9, 0.3, 0.3));

  // child shift points
  addAvoidLine(debug.merged, "c_0_merged", 0.345, 0.968, 1.0);
  addAvoidLine(debug.trim_similar_grad_shift, "c_1_trim_similar_grad_shift", 0.976, 0.328, 0.910);
  addAvoidLine(debug.quantized, "c_2_quantized", 0.505, 0.745, 0.969);
  addAvoidLine(debug.trim_small_shift, "c_3_trim_small_shift", 0.663, 0.525, 0.941);
  addAvoidLine(
    debug.trim_similar_grad_shift_second, "c_4_trim_similar_grad_shift", 0.97, 0.32, 0.91);
  addAvoidLine(debug.trim_momentary_return, "c_5_trim_momentary_return", 0.976, 0.078, 0.878);
  addAvoidLine(debug.trim_too_sharp_shift, "c_6_trim_too_sharp_shift", 0.576, 0.0, 0.978);

  addShiftLine(shifter.getShiftLines(), "path_shifter_registered_points", 0.99, 0.99, 0.0, 0.5);
  addAvoidLine(debug.new_shift_lines, "path_shifter_proposed_points", 0.99, 0.0, 0.0, 0.5);
}

void DynamicAvoidanceModule::updateDynamicAvoidanceDebugData(
  std::vector<DynamicAvoidanceDebugMsg> & avoidance_debug_msg_array) const
{
  debug_data_.avoidance_debug_msg_array.avoidance_info.clear();
  auto & debug_data_avoidance = debug_data_.avoidance_debug_msg_array.avoidance_info;
  debug_data_avoidance = avoidance_debug_msg_array;
  if (!debug_avoidance_initializer_for_shift_line_.empty()) {
    const bool is_info_old_ =
      (clock_->now() - debug_avoidance_initializer_for_shift_line_time_).seconds() > 0.1;
    if (!is_info_old_) {
      debug_data_avoidance.insert(
        debug_data_avoidance.end(), debug_avoidance_initializer_for_shift_line_.begin(),
        debug_avoidance_initializer_for_shift_line_.end());
    }
  }
}

double DynamicAvoidanceModule::getFeasibleDecelDistance(const double target_velocity) const
{
  const auto & a_now = planner_data_->self_acceleration->accel.accel.linear.x;
  const auto & a_lim = parameters_->max_deceleration;
  const auto & j_lim = parameters_->max_jerk;
  return calcDecelDistWithJerkAndAccConstraints(
    getEgoSpeed(), target_velocity, a_now, a_lim, j_lim, -1.0 * j_lim);
}

double DynamicAvoidanceModule::getMildDecelDistance(const double target_velocity) const
{
  const auto & a_now = planner_data_->self_acceleration->accel.accel.linear.x;
  const auto & a_lim = parameters_->nominal_deceleration;
  const auto & j_lim = parameters_->nominal_jerk;
  return calcDecelDistWithJerkAndAccConstraints(
    getEgoSpeed(), target_velocity, a_now, a_lim, j_lim, -1.0 * j_lim);
}

double DynamicAvoidanceModule::getRelativeLengthFromPath(const AvoidLine & avoid_line) const
{
  if (prev_reference_.points.size() != prev_linear_shift_path_.path.points.size()) {
    throw std::logic_error("prev_reference_ and prev_linear_shift_path_ must have same size.");
  }

  const auto current_shift_length = prev_linear_shift_path_.shift_length.at(
    findNearestIndex(prev_reference_.points, avoid_line.end.position));

  return avoid_line.end_shift_length - current_shift_length;
}

void DynamicAvoidanceModule::insertWaitPoint(
  const bool use_constraints_for_decel, ShiftedPath & shifted_path) const
{
  const auto & p = parameters_;
  const auto & data = avoidance_data_;
  const auto & base_link2front = planner_data_->parameters.base_link2front;
  const auto & vehicle_width = planner_data_->parameters.vehicle_width;

  if (!data.stop_target_object) {
    return;
  }

  if (data.avoiding_now) {
    return;
  }

  //         D5
  //      |<---->|                               D4
  //      |<----------------------------------------------------------------------->|
  // +-----------+            D1                 D2                      D3         +-----------+
  // |           |        |<------->|<------------------------->|<----------------->|           |
  // |    ego    |======= x ======= x ========================= x ==================|    obj    |
  // |           |    stop_point  avoid                       avoid                 |           |
  // +-----------+                start                        end                  +-----------+
  //
  // D1: p.min_prepare_distance
  // D2: min_avoid_distance
  // D3: longitudinal_avoid_margin_front (margin + D5)
  // D4: o_front.longitudinal
  // D5: base_link2front

  const auto o_front = data.stop_target_object.get();

  const auto avoid_margin =
    p->lateral_collision_safety_buffer + p->lateral_collision_margin + 0.5 * vehicle_width;
  const auto variable =
    getMinimumDynamicAvoidanceDistance(getShiftLength(o_front, isOnRight(o_front), avoid_margin));
  const auto constant =
    p->min_prepare_distance + p->longitudinal_collision_safety_buffer + base_link2front;
  const auto start_longitudinal =
    o_front.longitudinal -
    std::clamp(variable + constant, p->stop_min_distance, p->stop_max_distance);

  if (!use_constraints_for_decel) {
    insertDecelPoint(
      getEgoPosition(), start_longitudinal, 0.0, shifted_path.path, debug_data_.stop_pose);
    return;
  }

  const auto stop_distance = getMildDecelDistance(0.0);

  const auto insert_distance = std::max(start_longitudinal, stop_distance);

  insertDecelPoint(
    getEgoPosition(), insert_distance, 0.0, shifted_path.path, debug_data_.stop_pose);
}

void DynamicAvoidanceModule::insertYieldVelocity(ShiftedPath & shifted_path) const
{
  const auto & p = parameters_;
  const auto & data = avoidance_data_;

  if (data.target_objects.empty()) {
    return;
  }

  if (data.avoiding_now) {
    return;
  }

  const auto decel_distance = getMildDecelDistance(p->yield_velocity);

  insertDecelPoint(
    getEgoPosition(), decel_distance, p->yield_velocity, shifted_path.path, debug_data_.slow_pose);
}

void DynamicAvoidanceModule::insertPrepareVelocity(
  const bool avoidable, ShiftedPath & shifted_path) const
{
  const auto & data = avoidance_data_;

  if (data.target_objects.empty()) {
    return;
  }

  if (!!data.stop_target_object) {
    if (data.stop_target_object.get().reason != DynamicAvoidanceDebugFactor::TOO_LARGE_JERK) {
      return;
    }
  }

  if (data.avoiding_now) {
    return;
  }

  if (avoidable) {
    return;
  }

  const auto decel_distance = getFeasibleDecelDistance(0.0);

  insertDecelPoint(getEgoPosition(), decel_distance, 0.0, shifted_path.path, debug_data_.slow_pose);
}

std::shared_ptr<DynamicAvoidanceDebugMsgArray> DynamicAvoidanceModule::get_debug_msg_array() const
{
  debug_data_.avoidance_debug_msg_array.header.stamp = clock_->now();
  return std::make_shared<DynamicAvoidanceDebugMsgArray>(debug_data_.avoidance_debug_msg_array);
}

void DynamicAvoidanceModule::acceptVisitor(
  const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitDynamicAvoidanceModule(this);
  }
}

void SceneModuleVisitor::visitDynamicAvoidanceModule(const DynamicAvoidanceModule * module) const
{
  avoidance_visitor_ = module->get_debug_msg_array();
}
}  // namespace behavior_path_planner
