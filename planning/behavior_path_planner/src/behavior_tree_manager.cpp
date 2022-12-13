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

#include "behavior_path_planner/behavior_tree_manager.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/scene_module_bt_node_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <algorithm>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_path_planner
{
BehaviorTreeManager::BehaviorTreeManager(
  rclcpp::Node & node, const BehaviorTreeManagerParam & param)
: bt_manager_param_(param),
  logger_(node.get_logger().get_child("behavior_tree_manager")),
  clock_(*node.get_clock())
{
}

void BehaviorTreeManager::createBehaviorTree()
{
  blackboard_ = BT::Blackboard::create();
  try {
    bt_tree_ = bt_factory_.createTreeFromFile(bt_manager_param_.bt_tree_config_path, blackboard_);
  } catch (...) {
    RCLCPP_ERROR(
      logger_, "Failed to create BT from: %s", bt_manager_param_.bt_tree_config_path.c_str());
    exit(EXIT_FAILURE);
  }
  addGrootMonitoring(
    &bt_tree_, bt_manager_param_.groot_zmq_publisher_port, bt_manager_param_.groot_zmq_server_port);
}

void BehaviorTreeManager::registerSceneModule(const std::shared_ptr<SceneModuleInterface> & module)
{
  const std::string & name = module->name();
  const auto status = std::make_shared<SceneModuleStatus>(name);

  // simple condition node for "isRequested"
  bt_factory_.registerSimpleCondition(name + "_Request", [module, status](BT::TreeNode &) {
    return isExecutionRequested(module, status);
  });

  // simple action node for "planCandidate"
  auto bt_node =
    std::make_shared<SceneModuleBTNodeInterface>("", BT::NodeConfiguration{}, module, status);
  bt_factory_.registerSimpleAction(
    name + "_PlanCandidate",
    [bt_node](BT::TreeNode & tree_node) { return bt_node->planCandidate(tree_node); },
    SceneModuleBTNodeInterface::providedPorts());

  // register builder with default tick functor for "plan"
  auto builder = [module, status](
                   const std::string & _name, const BT::NodeConfiguration & _config) {
    return std::make_unique<SceneModuleBTNodeInterface>(_name, _config, module, status);
  };
  bt_factory_.registerBuilder<SceneModuleBTNodeInterface>(name + "_Plan", builder);

  scene_modules_.push_back(module);
  modules_status_.push_back(status);
}

BehaviorModuleOutput BehaviorTreeManager::run(const std::shared_ptr<PlannerData> & data)
{
  current_planner_data_ = data;

  // set planner_data & reset status
  std::for_each(
    scene_modules_.begin(), scene_modules_.end(), [&data](const auto & m) { m->setData(data); });
  std::for_each(modules_status_.begin(), modules_status_.end(), [](const auto & s) {
    *s = SceneModuleStatus{s->module_name};
  });

  // reset blackboard
  blackboard_->set<BehaviorModuleOutput>("output", BehaviorModuleOutput{});

  const auto res = bt_tree_.tickRoot();

  const auto output = blackboard_->get<BehaviorModuleOutput>("output");

  RCLCPP_DEBUG(logger_, "BehaviorPathPlanner::run end status = %s", BT::toStr(res).c_str());

  publishPathCandidate();

  std::for_each(scene_modules_.begin(), scene_modules_.end(), [](const auto & m) {
    if (!m->isExecutionRequested()) {
      m->onExit();
    }
    m->publishRTCStatus();
  });
  return output;
}

std::vector<std::shared_ptr<behavior_path_planner::SceneModuleStatus>>
BehaviorTreeManager::getModulesStatus()
{
  return modules_status_;
}

std::vector<MarkerArray> BehaviorTreeManager::getDebugMarkers()
{
  std::vector<MarkerArray> data;
  for (const auto & module : scene_modules_) {
    data.push_back(module->getDebugMarker());
  }
  return data;
}

BT::NodeStatus BehaviorTreeManager::checkForceApproval(const std::string & name)
{
  const auto & approval = current_planner_data_->approval.is_force_approved;
  if ((clock_.now() - approval.stamp).seconds() > 1.0) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 3000, "BehaviorTreeManager : Force approval data is time out!");
    return BT::NodeStatus::FAILURE;
  }

  return approval.module_name == name ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void BehaviorTreeManager::publishPathCandidate()
{
  const bool is_running = std::any_of(
    scene_modules_.begin(), scene_modules_.end(),
    [](const auto & module) { return module->current_state_ == BT::NodeStatus::RUNNING; });

  for (auto & module : scene_modules_) {
    if (is_running && (module->current_state_ != BT::NodeStatus::RUNNING)) {
      module->resetPathCandidate();
    }
    module->publishPathCandidate(
      convertToPath(module->getPathCandidate(), module->isExecutionReady()));
  }
}

Path BehaviorTreeManager::convertToPath(
  const std::shared_ptr<PathWithLaneId> & path_candidate_ptr, const bool is_ready)
{
  Path output;
  output.header = current_planner_data_->route_handler->getRouteHeader();
  output.header.stamp = clock_.now();

  if (!path_candidate_ptr) {
    return output;
  }

  output = util::toPath(*path_candidate_ptr);

  if (!is_ready) {
    for (auto & point : output.points) {
      point.longitudinal_velocity_mps = 0.0;
    }
  }

  return output;
}

void BehaviorTreeManager::resetBehaviorTree() { bt_tree_.haltTree(); }

void BehaviorTreeManager::addGrootMonitoring(
  BT::Tree * tree, uint16_t publisher_port, uint16_t server_port, uint16_t max_msg_per_second)
{
  groot_monitor_ =
    std::make_unique<BT::PublisherZMQ>(*tree, max_msg_per_second, publisher_port, server_port);
}

void BehaviorTreeManager::resetGrootMonitor()
{
  if (groot_monitor_) {
    groot_monitor_.reset();
  }
}

void BehaviorTreeManager::get_debug_msg_array()
{
  bt_visitor_interface_ = std::make_shared<BehaviorTreeVisitorInterface>();
  for (const auto & module : scene_modules_) {
    if (module->name() == "LaneChange") {
      auto visitor_ptr = std::make_shared<LaneChangeVisitor>();
      module->accept_visitor(visitor_ptr);
      const auto get_ptr = visitor_ptr->get_lane_change_debug_msg_array();
      if (get_ptr) {
        bt_visitor_interface_->set_lane_change_debug_ptr(get_ptr);
      }
    } else if (module->name() == "Avoidance") {
      auto visitor_ptr = std::make_shared<AvoidanceVisitor>();
      module->accept_visitor(visitor_ptr);
      const auto get_ptr = visitor_ptr->get_avoidance_debug_msg_array();
      if (get_ptr) {
        bt_visitor_interface_->set_avoidance_debug_ptr(get_ptr);
      }
    }
  }
}

std::shared_ptr<BehaviorTreeVisitorInterface> BehaviorTreeManager::get_all_debug_data()
{
  get_debug_msg_array();
  return bt_visitor_interface_;
}
}  // namespace behavior_path_planner
