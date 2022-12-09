// Copyright 2022 Tier IV, Inc.
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

#ifndef SAMPLER_NODE__TRAJECTORY_GENERATION_HPP_
#define SAMPLER_NODE__TRAJECTORY_GENERATION_HPP_

#include "bezier_sampler/bezier_sampling.hpp"
#include "frenet_planner/structures.hpp"
#include "sampler_common/constraints/hard_constraint.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/gui/gui.hpp"
#include "sampler_node/parameters.hpp"

#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <vector>

namespace sampler_node
{
/**
 * @brief generate candidate paths for the given problem inputs
 * @param [in] initial_state initial ego state
 * @param [in] path reference path to follow
 * @param [in] path_spline spline of the reference path
 * @param [in] previous_path previous path followed by ego
 * @param [in] constraints hard and soft constraints of the problem
 * @return generated candidate paths
 */
std::vector<sampler_common::Trajectory> generateCandidateTrajectories(
  const sampler_common::Configuration & initial_configuration,
  const sampler_common::Trajectory & previous_trajectory,
  const sampler_common::transform::Spline2D & path_spline,
  const autoware_auto_planning_msgs::msg::Path & path_msg, gui::GUI & gui,
  const Parameters & params);

std::vector<frenet_planner::Trajectory> generateFrenetTrajectories(
  const sampler_common::Configuration & initial_configuration,
  const sampler_common::Trajectory & base_traj,
  const autoware_auto_planning_msgs::msg::Path & path_msg,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params);
}  // namespace sampler_node

#endif  // SAMPLER_NODE__TRAJECTORY_GENERATION_HPP_