// Copyright (c) 2018 Intel Corporation
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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/find_docking_point_action.hpp"

namespace nav2_behavior_tree
{
FindDockingPointAction::FindDockingPointAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::FindDockingPoint>(xml_tag_name, action_name, conf)
{
  double distance_to_point;
  getInput("distance_to_point", distance_to_point);
  goal_.distance_to_point = distance_to_point;
}

void FindDockingPointAction::on_tick()
{

}

BT::NodeStatus FindDockingPointAction::on_success()
{
  setOutput("docking_point", result_.result->docking_point);
      RCLCPP_INFO(
      node_->get_logger(),
      "FindDockingPointAction: found point y: %f y: %f", result_.result->docking_point.pose.position.x, result_.result->docking_point.pose.position.y);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FindDockingPointAction::on_aborted()
{
  geometry_msgs::msg::PoseStamped point;
  setOutput("docking_point", point);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus FindDockingPointAction::on_cancelled()
{
  geometry_msgs::msg::PoseStamped point;
  setOutput("docking_point", point);
  return BT::NodeStatus::SUCCESS;
}

void FindDockingPointAction::halt()
{
  geometry_msgs::msg::PoseStamped point;
  setOutput("docking_point", point);
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::FindDockingPointAction>(
        name, "find_docking_point", config);
    };

  factory.registerBuilder<nav2_behavior_tree::FindDockingPointAction>("FindDockingPoint", builder);
}
