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

#include "nav2_behavior_tree/plugins/action/precise_nav_action.hpp"

namespace nav2_behavior_tree
{
PreciseNavAction::PreciseNavAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::PreciseNav>(xml_tag_name, action_name, conf)
{}

void PreciseNavAction::on_tick()
{
  if (!getInput("goal", goal_.pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "NavigateToPoseAction: goal not provided");
    return;
  }
  getInput("goal", goal_.pose);
}

BT::NodeStatus PreciseNavAction::on_success()
{
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PreciseNavAction::on_aborted()
{
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PreciseNavAction::on_cancelled()
{
  return BT::NodeStatus::SUCCESS;
}

void PreciseNavAction::halt()
{
  BtActionNode::halt();
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::PreciseNavAction>(
        name, "precise_nav", config);
    };

  factory.registerBuilder<nav2_behavior_tree::PreciseNavAction>("PreciseNav", builder);
}
