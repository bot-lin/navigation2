// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
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

#include "nav2_behavior_tree/plugins/condition/is_fork_loaded_condition.hpp"

namespace nav2_behavior_tree
{

IsForkLoadedCondition::IsForkLoadedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  forkloaded_topic_("forkloaded"),
  fork_loaded_(false)
{
  getInput("forkloaded_topic", forkloaded_topic_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  fork_loaded_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    forkloaded_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsForkLoadedCondition::forkLoadCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsForkLoadedCondition::tick()
{
  callback_group_executor_.spin_some();

  if (fork_loaded_) 
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
  
}

void IsForkLoadedCondition::forkLoadCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  fork_loaded_ = msg->data;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsForkLoadedCondition>("IsForkLoaded");
}
