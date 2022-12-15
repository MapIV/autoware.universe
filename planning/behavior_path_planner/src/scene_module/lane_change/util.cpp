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

#include "behavior_path_planner/scene_module/lane_change/util.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner::lane_change_utils
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using lanelet::ArcCoordinates;

PathWithLaneId combineReferencePath(const PathWithLaneId & path1, const PathWithLaneId & path2)
{
  PathWithLaneId path;
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  // skip overlapping point
  path.points.insert(path.points.end(), next(path2.points.begin()), path2.points.end());

  return path;
}

bool isPathInLanelets(
  const PathWithLaneId & path, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets)
{
  for (const auto & pt : path.points) {
    bool is_in_lanelet = false;
    for (const auto & llt : original_lanelets) {
      if (lanelet::utils::isInLanelet(pt.point.pose, llt, 0.1)) {
        is_in_lanelet = true;
      }
    }
    for (const auto & llt : target_lanelets) {
      if (lanelet::utils::isInLanelet(pt.point.pose, llt, 0.1)) {
        is_in_lanelet = true;
      }
    }
    if (!is_in_lanelet) {
      return false;
    }
  }
  return true;
}
double getExpectedVelocityWhenDecelerate(
  const double & velocity, const double & expected_acceleration, const double & duration)
{
  return velocity + expected_acceleration * duration;
}

double getDistanceWhenDecelerate(
  const double & velocity, const double & expected_acceleration, const double & duration,
  const double & minimum_distance)
{
  const auto distance = velocity * duration + 0.5 * expected_acceleration * std::pow(duration, 2);
  return std::max(distance, minimum_distance);
}

std::pair<double, double> calcLaneChangingSpeedAndDistanceWhenDecelerate(
  const double velocity, const double shift_length, const double acceleration,
  const double minimum_lane_change_length, const LaneChangeParameters & params)
{
  const auto required_time = PathShifter::calcShiftTimeFromJerkAndJerk(
    shift_length, params.lane_changing_lateral_jerk, params.lane_changing_lateral_acc);

  const auto lane_changing_average_speed =
    std::max(velocity + acceleration * 0.5 * required_time, params.minimum_lane_change_velocity);
  const auto lane_changing_distance =
    std::max(lane_changing_average_speed * required_time, minimum_lane_change_length);

  return {lane_changing_average_speed, lane_changing_distance};
}

std::optional<LaneChangePath> constructCandidatePath(
  const PathWithLaneId & prepare_segment, const PathWithLaneId & lane_changing_segment,
  const PathWithLaneId & target_lane_reference_path, const ShiftPoint & shift_point,
  const lanelet::ConstLanelets & original_lanelets, const lanelet::ConstLanelets & target_lanelets,
  const double & acceleration, const double & prepare_distance,
  [[maybe_unused]] const double & prepare_duration, [[maybe_unused]] const double & prepare_speed,
  const double & lane_change_distance, const double & lane_changing_speed,
  const LaneChangeParameters & params)
{
  const auto resampled_reference_path = target_lane_reference_path;

  PathShifter path_shifter;
  path_shifter.setPath(resampled_reference_path);
  path_shifter.addShiftPoint(shift_point);
  ShiftedPath shifted_path;

  // offset front side
  bool offset_back = false;

  path_shifter.setVelocity(lane_changing_speed);
  path_shifter.setLateralAccelerationLimit(std::abs(params.lane_changing_lateral_acc));

  if (!path_shifter.generate(&shifted_path, offset_back)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
      "failed to generate shifted path.");
  }

  LaneChangePath candidate_path;
  candidate_path.acceleration = acceleration;
  candidate_path.preparation_length = prepare_distance;
  candidate_path.lane_change_length = lane_change_distance;
  candidate_path.shift_point = shift_point;
  candidate_path.reference_lanelets = original_lanelets;
  candidate_path.target_lanelets = target_lanelets;

  const PathPointWithLaneId & lane_changing_start_point = prepare_segment.points.back();
  const PathPointWithLaneId & lane_changing_end_point = lane_changing_segment.points.front();
  const Pose & lane_changing_end_pose = lane_changing_end_point.point.pose;
  const auto lanechange_end_idx =
    motion_utils::findNearestIndex(shifted_path.path.points, lane_changing_end_pose);
  const auto insertLaneIDs = [](auto & target, const auto src) {
    target.lane_ids.insert(target.lane_ids.end(), src.lane_ids.begin(), src.lane_ids.end());
  };
  if (lanechange_end_idx) {
    for (size_t i = 0; i < shifted_path.path.points.size(); ++i) {
      auto & point = shifted_path.path.points.at(i);
      if (i < *lanechange_end_idx) {
        insertLaneIDs(point, lane_changing_start_point);
        insertLaneIDs(point, lane_changing_end_point);
        point.point.longitudinal_velocity_mps = std::min(
          point.point.longitudinal_velocity_mps,
          lane_changing_start_point.point.longitudinal_velocity_mps);
        continue;
      }
      point.point.longitudinal_velocity_mps =
        std::min(point.point.longitudinal_velocity_mps, static_cast<float>(lane_changing_speed));
      const auto nearest_idx =
        motion_utils::findNearestIndex(lane_changing_segment.points, point.point.pose);
      point.lane_ids = lane_changing_segment.points.at(*nearest_idx).lane_ids;
    }

    candidate_path.path = combineReferencePath(prepare_segment, shifted_path.path);
    candidate_path.shifted_path = shifted_path;
  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
      "lane change end idx not found on target path.");
    return std::nullopt;
  }

  // check candidate path is in lanelet
  if (!isPathInLanelets(candidate_path.path, original_lanelets, target_lanelets)) {
    return std::nullopt;
  }

  return std::optional<LaneChangePath>{candidate_path};
}

LaneChangePaths getLaneChangePaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets, const Pose & pose, const Twist & twist,
  const BehaviorPathPlannerParameters & common_parameter, const LaneChangeParameters & parameter)
{
  LaneChangePaths candidate_paths{};

  if (original_lanelets.empty() || target_lanelets.empty()) {
    return candidate_paths;
  }

  // rename parameter
  const auto & backward_path_length = common_parameter.backward_path_length;
  const auto & forward_path_length = common_parameter.forward_path_length;
  const auto & lane_change_prepare_duration = parameter.lane_change_prepare_duration;
  const auto & minimum_lane_change_prepare_distance =
    common_parameter.minimum_lane_change_prepare_distance;
  const auto & minimum_lane_change_length = common_parameter.minimum_lane_change_length;
  const auto & minimum_lane_change_velocity = parameter.minimum_lane_change_velocity;
  const auto & maximum_deceleration = parameter.maximum_deceleration;
  const auto & lane_change_sampling_num = parameter.lane_change_sampling_num;
  const auto & backward_length_buffer = common_parameter.backward_length_buffer_for_end_of_lane;

  const int num_to_preferred_lane =
    std::abs(route_handler.getNumLaneToPreferredLane(target_lanelets.back()));

  // get velocity
  const double current_velocity = util::l2Norm(twist.linear);

  constexpr double buffer = 1.0;  // buffer for min_lane_change_length
  const double acceleration_resolution = std::abs(maximum_deceleration) / lane_change_sampling_num;

  const double target_distance =
    util::getArcLengthToTargetLanelet(original_lanelets, target_lanelets.front(), pose);

  for (double acceleration = 0.0; acceleration >= -maximum_deceleration;
       acceleration -= acceleration_resolution) {
    const double prepare_speed = getExpectedVelocityWhenDecelerate(
      current_velocity, acceleration, lane_change_prepare_duration);

    // skip if velocity becomes less than zero before starting lane change
    if (prepare_speed < 0.0) {
      continue;
    }

    // get path on original lanes
    const double prepare_distance = getDistanceWhenDecelerate(
      current_velocity, acceleration, lane_change_prepare_duration,
      minimum_lane_change_prepare_distance);

    if (prepare_distance < target_distance) {
      continue;
    }

    const PathWithLaneId prepare_segment_reference = getLaneChangePathPrepareSegment(
      route_handler, original_lanelets, pose, backward_path_length, prepare_distance,
      lane_change_prepare_duration, minimum_lane_change_velocity);

    const auto estimated_shift_length = lanelet::utils::getArcCoordinates(
      target_lanelets, prepare_segment_reference.points.front().point.pose);
    const auto [lane_changing_speed, lane_changing_distance] =
      calcLaneChangingSpeedAndDistanceWhenDecelerate(
        prepare_speed, estimated_shift_length.distance, acceleration, minimum_lane_change_length,
        parameter);

    const PathWithLaneId lane_changing_segment_reference = getLaneChangePathLaneChangingSegment(
      route_handler, target_lanelets, pose, forward_path_length, prepare_distance,
      lane_changing_distance, minimum_lane_change_length, buffer, lane_changing_speed);

    if (
      prepare_segment_reference.points.empty() || lane_changing_segment_reference.points.empty()) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
        "reference path is empty!! something wrong...");
      continue;
    }

    const Pose & lane_changing_start_pose = prepare_segment_reference.points.back().point.pose;

    const PathWithLaneId target_lane_reference_path = getReferencePathFromTargetLane(
      route_handler, target_lanelets, lane_changing_start_pose, prepare_distance,
      lane_changing_distance, forward_path_length, num_to_preferred_lane,
      minimum_lane_change_length + backward_length_buffer, lane_changing_speed);

    const ShiftPoint shift_point = getLaneChangeShiftPoint(
      prepare_segment_reference, lane_changing_segment_reference, target_lanelets,
      target_lane_reference_path);

    const auto candidate_path = constructCandidatePath(
      prepare_segment_reference, lane_changing_segment_reference, target_lane_reference_path,
      shift_point, original_lanelets, target_lanelets, acceleration, prepare_distance,
      lane_change_prepare_duration, prepare_speed, lane_changing_distance, lane_changing_speed,
      parameter);

    if (!candidate_path) {
      continue;
    }

    candidate_paths.push_back(*candidate_path);
  }

  return candidate_paths;
}

LaneChangePaths selectValidPaths(
  const LaneChangePaths & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const RouteHandler & route_handler,
  const Pose & current_pose, const Pose & goal_pose, const double minimum_lane_change_length)
{
  LaneChangePaths available_paths;

  for (const auto & path : paths) {
    if (hasEnoughDistance(
          path, current_lanes, target_lanes, current_pose, goal_pose, route_handler,
          minimum_lane_change_length)) {
      available_paths.push_back(path);
    }
  }

  return available_paths;
}

bool selectSafePath(
  const LaneChangePaths & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const PredictedObjects::ConstSharedPtr dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & ros_parameters, LaneChangePath * selected_path,
  std::unordered_map<std::string, CollisionCheckDebug> & debug_data)
{
  debug_data.clear();
  for (const auto & path : paths) {
    Pose ego_pose_before_collision;
    if (isLaneChangePathSafe(
          path.path, current_lanes, target_lanes, dynamic_objects, current_pose, current_twist,
          common_parameters, ros_parameters, common_parameters.expected_front_deceleration,
          common_parameters.expected_rear_deceleration, ego_pose_before_collision, debug_data, true,
          path.acceleration)) {
      *selected_path = path;
      return true;
    }
  }
  // set first path for force lane change if no valid path found
  if (!paths.empty()) {
    *selected_path = paths.front();
    return false;
  }

  return false;
}
bool hasEnoughDistance(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
  [[maybe_unused]] const lanelet::ConstLanelets & target_lanes, const Pose & current_pose,
  const Pose & goal_pose, const RouteHandler & route_handler,
  const double minimum_lane_change_length)
{
  const double & lane_change_prepare_distance = path.preparation_length;
  const double & lane_changing_distance = path.lane_change_length;
  const double lane_change_total_distance = lane_change_prepare_distance + lane_changing_distance;
  const int num = std::abs(route_handler.getNumLaneToPreferredLane(target_lanes.back()));
  const auto overall_graphs = route_handler.getOverallGraphPtr();

  const double lane_change_required_distance =
    static_cast<double>(num) * minimum_lane_change_length;

  if (
    lane_change_total_distance + lane_change_required_distance >
    util::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return false;
  }

  if (
    lane_change_total_distance + lane_change_required_distance >
    util::getDistanceToNextIntersection(current_pose, current_lanes)) {
    return false;
  }

  if (
    route_handler.isInGoalRouteSection(current_lanes.back()) &&
    lane_change_total_distance + lane_change_required_distance >
      util::getSignedDistance(current_pose, goal_pose, current_lanes)) {
    return false;
  }

  if (
    lane_change_total_distance + lane_change_required_distance >
    util::getDistanceToCrosswalk(current_pose, current_lanes, *overall_graphs)) {
    return false;
  }

  // return is there is no target lanes. Else continue checking
  if (target_lanes.empty()) {
    return true;
  }

  return true;
}

bool isLaneChangePathSafe(
  const PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const PredictedObjects::ConstSharedPtr dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters, const double front_decel,
  const double rear_decel, Pose & ego_pose_before_collision,
  std::unordered_map<std::string, CollisionCheckDebug> & debug_data, const bool use_buffer,
  const double acceleration)
{
  if (dynamic_objects == nullptr) {
    return true;
  }

  if (path.points.empty() || target_lanes.empty() || current_lanes.empty()) {
    return false;
  }

  const double time_resolution = lane_change_parameters.prediction_time_resolution;
  const auto & lane_change_prepare_duration = lane_change_parameters.lane_change_prepare_duration;
  const auto & enable_collision_check_at_prepare_phase =
    lane_change_parameters.enable_collision_check_at_prepare_phase;
  const auto & lane_changing_safety_check_duration =
    lane_change_parameters.lane_changing_safety_check_duration;
  const double check_end_time = lane_change_prepare_duration + lane_changing_safety_check_duration;
  constexpr double ego_predicted_path_min_speed{1.0};
  const auto vehicle_predicted_path = util::convertToPredictedPath(
    path, current_twist, current_pose, check_end_time, time_resolution, acceleration,
    ego_predicted_path_min_speed);

  const auto arc = lanelet::utils::getArcCoordinates(current_lanes, current_pose);

  // find obstacle in lane change target lanes
  // retrieve lanes that are merging target lanes as well
  const auto target_lane_object_indices =
    util::filterObjectsByLanelets(*dynamic_objects, target_lanes);

  // find objects in current lane
  constexpr double check_distance = 100.0;
  const auto current_lane_object_indices_lanelet = util::filterObjectsByLanelets(
    *dynamic_objects, current_lanes, arc.length, arc.length + check_distance);

  const double lateral_buffer = (use_buffer) ? 0.5 : 0.0;
  const auto & vehicle_info = common_parameters.vehicle_info;
  const auto & vehicle_width = common_parameters.vehicle_width;
  // const auto & vehicle_length = common_parameters.vehicle_length;
  const auto current_lane_object_indices = util::filterObjectsByPath(
    *dynamic_objects, current_lane_object_indices_lanelet, path,
    vehicle_width / 2 + lateral_buffer);

  const auto assignDebugData = [](const PredictedObject & obj) {
    CollisionCheckDebug debug;
    const auto key = util::getUuidStr(obj);
    debug.current_pose = obj.kinematics.initial_pose_with_covariance.pose;
    debug.current_twist = obj.kinematics.initial_twist_with_covariance.twist;
    return std::make_pair(key, debug);
  };

  const auto appendDebugInfo =
    [&debug_data](std::pair<std::string, CollisionCheckDebug> & obj, bool && is_allowed) {
      const auto & key = obj.first;
      auto & element = obj.second;
      element.allow_lane_change = is_allowed;
      if (debug_data.find(key) != debug_data.end()) {
        debug_data[key] = element;
      } else {
        debug_data.insert(obj);
      }
    };

  for (const auto & i : current_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    const auto object_speed =
      util::l2Norm(obj.kinematics.initial_twist_with_covariance.twist.linear);
    const double check_start_time =
      (enable_collision_check_at_prepare_phase && (object_speed > 0.1))
        ? 0.0
        : lane_change_prepare_duration;
    auto current_debug_data = assignDebugData(obj);
    const auto predicted_paths =
      util::getPredictedPathFromObj(obj, lane_change_parameters.use_all_predicted_path);
    for (const auto & obj_path : predicted_paths) {
      if (!util::isSafeInLaneletCollisionCheck(
            current_pose, current_twist, vehicle_predicted_path, vehicle_info, check_start_time,
            check_end_time, time_resolution, obj, obj_path, common_parameters, front_decel,
            rear_decel, ego_pose_before_collision, current_debug_data.second)) {
        appendDebugInfo(current_debug_data, false);
        return false;
      }
    }
  }

  // Collision check for objects in lane change target lane
  for (const auto & i : target_lane_object_indices) {
    const auto & obj = dynamic_objects->objects.at(i);
    auto current_debug_data = assignDebugData(obj);
    current_debug_data.second.ego_predicted_path.push_back(vehicle_predicted_path);
    bool is_object_in_target = false;
    if (lane_change_parameters.use_predicted_path_outside_lanelet) {
      is_object_in_target = true;
    } else {
      for (const auto & llt : target_lanes) {
        if (lanelet::utils::isInLanelet(obj.kinematics.initial_pose_with_covariance.pose, llt)) {
          is_object_in_target = true;
        }
      }
    }

    const auto predicted_paths =
      util::getPredictedPathFromObj(obj, lane_change_parameters.use_all_predicted_path);

    const auto object_speed =
      util::l2Norm(obj.kinematics.initial_twist_with_covariance.twist.linear);
    const double check_start_time =
      (enable_collision_check_at_prepare_phase && (object_speed > 0.1))
        ? 0.0
        : lane_change_prepare_duration;
    if (is_object_in_target) {
      for (const auto & obj_path : predicted_paths) {
        if (!util::isSafeInLaneletCollisionCheck(
              current_pose, current_twist, vehicle_predicted_path, vehicle_info, check_start_time,
              check_end_time, time_resolution, obj, obj_path, common_parameters, front_decel,
              rear_decel, ego_pose_before_collision, current_debug_data.second)) {
          appendDebugInfo(current_debug_data, false);
          return false;
        }
      }
    } else {
      if (!util::isSafeInFreeSpaceCollisionCheck(
            current_pose, current_twist, vehicle_predicted_path, vehicle_info, check_start_time,
            check_end_time, time_resolution, obj, common_parameters, front_decel, rear_decel,
            current_debug_data.second)) {
        appendDebugInfo(current_debug_data, false);
        return false;
      }
    }
    appendDebugInfo(current_debug_data, true);
  }
  return true;
}

PathWithLaneId getReferencePathFromTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & lane_changing_start_pose, const double & prepare_distance,
  const double & lane_changing_distance, const double & forward_path_length,
  const int & num_to_preferred_lane, const double & minimum_lane_change_length,
  const double & lane_changing_speed)
{
  const ArcCoordinates lane_change_start_arc_position =
    lanelet::utils::getArcCoordinates(target_lanes, lane_changing_start_pose);

  const double total_minimum_lane_change_length =
    static_cast<double>(num_to_preferred_lane) * minimum_lane_change_length;
  const double & s_start = lane_change_start_arc_position.length;
  double s_end = s_start;
  const double distance_to_deadend =
    util::getDistanceToEndOfLane(lane_changing_start_pose, target_lanes) -
    total_minimum_lane_change_length;

  if (num_to_preferred_lane == 0) {
    s_end += prepare_distance + lane_changing_distance + forward_path_length;
  } else {
    s_end += distance_to_deadend;
  }

  if (route_handler.isInGoalRouteSection(target_lanes.back())) {
    const auto goal_arc_coordinates =
      lanelet::utils::getArcCoordinates(target_lanes, route_handler.getGoalPose());
    s_end = std::min(s_end, goal_arc_coordinates.length - total_minimum_lane_change_length);
  }
  const auto lane_changing_reference_path =
    route_handler.getCenterLinePath(target_lanes, s_start, s_end);

  constexpr auto min_resampling_points{30.0};
  constexpr auto resampling_dt{0.2};
  const auto resample_interval =
    std::max(distance_to_deadend / min_resampling_points, lane_changing_speed * resampling_dt);
  return util::resamplePathWithSpline(lane_changing_reference_path, resample_interval);
}

PathWithLaneId getReferencePathFromTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & in_target_front_pose, const Pose & in_target_end_pose)
{
  const ArcCoordinates lane_change_start_arc_position =
    lanelet::utils::getArcCoordinates(target_lanes, in_target_front_pose);

  const ArcCoordinates lane_change_end_arc_position =
    lanelet::utils::getArcCoordinates(target_lanes, in_target_end_pose);

  const double & s_start = lane_change_start_arc_position.length;
  const double & s_end = lane_change_end_arc_position.length;
  return route_handler.getCenterLinePath(target_lanes, s_start, s_end);
}

ShiftPoint getLaneChangeShiftPoint(
  const PathWithLaneId & path1, const PathWithLaneId & path2,
  const lanelet::ConstLanelets & target_lanes, const PathWithLaneId & reference_path)
{
  const Pose & lane_change_start_on_self_lane = path1.points.back().point.pose;
  const Pose & lane_change_end_on_target_lane = path2.points.front().point.pose;
  const ArcCoordinates lane_change_start_on_self_lane_arc =
    lanelet::utils::getArcCoordinates(target_lanes, lane_change_start_on_self_lane);

  ShiftPoint shift_point;
  shift_point.length = lane_change_start_on_self_lane_arc.distance;
  shift_point.start = lane_change_start_on_self_lane;
  shift_point.end = lane_change_end_on_target_lane;
  shift_point.start_idx =
    motion_utils::findNearestIndex(reference_path.points, lane_change_start_on_self_lane.position);
  shift_point.end_idx =
    motion_utils::findNearestIndex(reference_path.points, lane_change_end_on_target_lane.position);

  return shift_point;
}

PathWithLaneId getLaneChangePathPrepareSegment(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const Pose & current_pose, const double & backward_path_length, const double & prepare_distance,
  const double & prepare_duration, const double & minimum_lane_change_velocity)
{
  if (original_lanelets.empty()) {
    return PathWithLaneId();
  }

  const ArcCoordinates arc_position =
    lanelet::utils::getArcCoordinates(original_lanelets, current_pose);
  const double s_start = arc_position.length - backward_path_length;
  const double s_end = arc_position.length + prepare_distance;

  PathWithLaneId prepare_segment =
    route_handler.getCenterLinePath(original_lanelets, s_start, s_end);

  prepare_segment.points.back().point.longitudinal_velocity_mps = std::min(
    prepare_segment.points.back().point.longitudinal_velocity_mps,
    static_cast<float>(
      std::max(prepare_distance / prepare_duration, minimum_lane_change_velocity)));

  return prepare_segment;
}

PathWithLaneId getLaneChangePathLaneChangingSegment(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanelets,
  const Pose & current_pose, const double & forward_path_length, const double & prepare_distance,
  const double & lane_change_distance, const double & minimum_lane_change_length,
  const double & lane_change_distance_buffer, const double & lane_changing_speed)
{
  if (target_lanelets.empty()) {
    return PathWithLaneId();
  }

  const ArcCoordinates arc_position =
    lanelet::utils::getArcCoordinates(target_lanelets, current_pose);
  const double lane_length = lanelet::utils::getLaneletLength2d(target_lanelets);
  const int num = std::abs(route_handler.getNumLaneToPreferredLane(target_lanelets.back()));

  const double s_start = std::invoke([&arc_position, &prepare_distance, &lane_change_distance,
                                      &lane_length, &num, &minimum_lane_change_length]() {
    const double s_start = arc_position.length + prepare_distance + lane_change_distance;
    return std::min(s_start, lane_length - num * minimum_lane_change_length);
  });

  const double s_end = std::invoke([&s_start, &forward_path_length, &lane_length, &num,
                                    &minimum_lane_change_length, &lane_change_distance_buffer]() {
    const double s_end = std::min(
      s_start + forward_path_length,
      lane_length - num * (minimum_lane_change_length + lane_change_distance_buffer));
    return std::max(s_end, s_start + std::numeric_limits<double>::epsilon());
  });

  PathWithLaneId lane_changing_segment =
    route_handler.getCenterLinePath(target_lanelets, s_start, s_end);
  for (auto & point : lane_changing_segment.points) {
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(lane_changing_speed));
  }

  return lane_changing_segment;
}
bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param)
{
  const auto lane_length = lanelet::utils::getLaneletLength2d(current_lanes);
  const auto lane_poly = lanelet::utils::getPolygonFromArcLength(current_lanes, 0, lane_length);
  const auto vehicle_poly =
    util::getVehiclePolygon(current_pose, common_param.vehicle_width, common_param.base_link2front);
  return boost::geometry::within(
    lanelet::utils::to2D(vehicle_poly).basicPolygon(),
    lanelet::utils::to2D(lane_poly).basicPolygon());
}

bool isEgoDistanceNearToCenterline(
  const lanelet::ConstLanelet & closest_lanelet, const Pose & current_pose,
  const LaneChangeParameters & lane_change_param)
{
  const auto centerline2d = lanelet::utils::to2D(closest_lanelet.centerline()).basicLineString();
  lanelet::BasicPoint2d vehicle_pose2d(current_pose.position.x, current_pose.position.y);
  const double distance = lanelet::geometry::distance2d(centerline2d, vehicle_pose2d);
  return distance < lane_change_param.abort_lane_change_distance_thresh;
}

bool isEgoHeadingAngleLessThanThreshold(
  const lanelet::ConstLanelet & closest_lanelet, const Pose & current_pose,
  const LaneChangeParameters & lane_change_param)
{
  const double lane_angle = lanelet::utils::getLaneletAngle(closest_lanelet, current_pose.position);
  const double vehicle_yaw = tf2::getYaw(current_pose.orientation);
  const double yaw_diff = tier4_autoware_utils::normalizeRadian(lane_angle - vehicle_yaw);
  return std::abs(yaw_diff) < lane_change_param.abort_lane_change_angle_thresh;
}

double abortPointDistance(
  const double starting_velocity, const double param_accel, const double param_jerk,
  const double param_time)
{
  return starting_velocity * param_time + param_accel * std::pow(param_time, 2) / 2. -
         param_jerk * std::pow(param_jerk, 3) / 6.;
}

std::optional<LaneChangePath> getAbortPaths(
  const std::shared_ptr<const PlannerData> & planner_data, const LaneChangePath & selected_path,
  [[maybe_unused]] const Pose & ego_pose_before_collision,
  const BehaviorPathPlannerParameters & common_param,
  [[maybe_unused]] const LaneChangeParameters & lane_change_param)
{
  const auto & route_handler = planner_data->route_handler;
  const auto current_speed = util::l2Norm(planner_data->self_odometry->twist.twist.linear);
  const auto current_pose = planner_data->self_pose->pose;
  const auto reference_lanelets = selected_path.reference_lanelets;

  const auto ego_nearest_dist_threshold = planner_data->parameters.ego_nearest_dist_threshold;
  const auto ego_nearest_yaw_threshold = planner_data->parameters.ego_nearest_yaw_threshold;
  const double minimum_lane_change_length = util::calcLaneChangeBuffer(common_param, 1, 0.0);

  const auto ego_pose_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    selected_path.path.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

  const auto lane_changing_end_pose_idx = std::invoke([&]() {
    constexpr double s_start = 0.0;
    const double s_end =
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_change_length;

    const auto ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end);
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      selected_path.path.points, ref.points.back().point.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
  });

  const auto pose_idx_min = [&](const double param_time) {
    double turning_point_dist{0.0};
    if (ego_pose_idx > lane_changing_end_pose_idx) {
      return std::make_pair(ego_pose_idx, turning_point_dist);
    }

    constexpr auto min_speed{2.77};
    const auto desired_distance = std::max(min_speed, current_speed) * param_time;
    const auto & points = selected_path.path.points;
    size_t idx{0};
    for (idx = ego_pose_idx; idx < lane_changing_end_pose_idx; ++idx) {
      const auto dist_to_ego =
        util::getSignedDistance(current_pose, points.at(idx).point.pose, reference_lanelets);
      turning_point_dist = dist_to_ego;
      if (dist_to_ego > desired_distance) {
        break;
      }
    }
    return std::make_pair(idx, turning_point_dist);
  };

  const auto abort_delta_time = lane_change_param.abort_delta_time;
  const auto [abort_start_idx, abort_start_dist] = pose_idx_min(abort_delta_time);
  const auto [abort_return_idx, abort_return_dist] = pose_idx_min(abort_delta_time * 2);

  if (abort_start_idx >= abort_return_idx) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
      "abort start idx and return idx is equal. can't compute abort path.");
    return std::nullopt;
  }

  if (!hasEnoughDistanceToLaneChangeAfterAbort(
        *route_handler, reference_lanelets, current_pose, abort_return_dist, common_param)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
      "insufficient distance to abort.");
    return std::nullopt;
  }

  const auto abort_start_pose = selected_path.path.points.at(abort_start_idx).point.pose;
  const auto abort_return_pose = selected_path.path.points.at(abort_return_idx).point.pose;

  const auto arc_position =
    lanelet::utils::getArcCoordinates(reference_lanelets, abort_return_pose);
  const PathWithLaneId reference_lane_segment = std::invoke([&]() {
    const double s_start = arc_position.length;
    const double s_end =
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_change_length;

    PathWithLaneId ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end);
    ref.points.back().point.longitudinal_velocity_mps = std::min(
      ref.points.back().point.longitudinal_velocity_mps,
      static_cast<float>(lane_change_param.minimum_lane_change_velocity));
    return ref;
  });

  ShiftPoint shift_point;
  shift_point.start = abort_start_pose;
  shift_point.end = abort_return_pose;
  shift_point.length = -arc_position.distance;
  shift_point.start_idx = abort_start_idx;
  shift_point.end_idx = abort_return_idx;

  PathShifter path_shifter;
  path_shifter.setPath(selected_path.path);
  path_shifter.addShiftPoint(shift_point);

  ShiftedPath shifted_path;
  // offset front side
  // bool offset_back = false;
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("lane_change").get_child("util"),
      "failed to generate abort shifted path.");
  }

  PathWithLaneId start_to_abort_end_pose;
  start_to_abort_end_pose.points.insert(
    start_to_abort_end_pose.points.end(), shifted_path.path.points.begin(),
    shifted_path.path.points.begin() + abort_return_idx);
  start_to_abort_end_pose.points.insert(
    start_to_abort_end_pose.points.end(), reference_lane_segment.points.begin(),
    reference_lane_segment.points.end());

  LaneChangePath abort_path(selected_path);
  abort_path.shifted_path = shifted_path;
  abort_path.shift_point = shift_point;
  abort_path.path = start_to_abort_end_pose;
  return std::optional<LaneChangePath>{abort_path};
}

double getLateralShift(const LaneChangePath & path)
{
  const auto start_idx = path.shift_point.start_idx;
  const auto end_idx = path.shift_point.end_idx;

  return path.shifted_path.shift_length.at(end_idx) - path.shifted_path.shift_length.at(start_idx);
}

bool hasEnoughDistanceToLaneChangeAfterAbort(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const Pose & current_pose, const double abort_return_dist,
  const BehaviorPathPlannerParameters & common_param)
{
  const auto minimum_lane_change_distance = common_param.minimum_lane_change_prepare_distance +
                                            common_param.minimum_lane_change_length +
                                            common_param.backward_length_buffer_for_end_of_lane;
  const auto abort_plus_lane_change_distance = abort_return_dist + minimum_lane_change_distance;
  if (abort_plus_lane_change_distance > util::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return false;
  }

  if (
    abort_plus_lane_change_distance >
    util::getDistanceToNextIntersection(current_pose, current_lanes)) {
    return false;
  }

  if (
    route_handler.isInGoalRouteSection(current_lanes.back()) &&
    abort_plus_lane_change_distance >
      util::getSignedDistance(current_pose, route_handler.getGoalPose(), current_lanes)) {
    return false;
  }

  if (
    abort_plus_lane_change_distance >
    util::getDistanceToCrosswalk(
      current_pose, current_lanes, *route_handler.getOverallGraphPtr())) {
    return false;
  }

  return true;
}
}  // namespace behavior_path_planner::lane_change_utils
