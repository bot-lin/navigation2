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

#include "nav2_behavior_tree/plugins/condition/is_charging_condition.hpp"

namespace nav2_behavior_tree
{

IsChargingCondition::IsChargingCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  charging_topic_("/charging_status"),
  is_charging_(true)
{
  getInput("charging_topic", charging_topic_);
  getInput("charging_duration", charing_duration_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "################################");
  RCLCPP_INFO(node_->get_logger(), "Starting is charging condition");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  charging_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
    charging_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&IsChargingCondition::chargingCallback, this, std::placeholders::_1),
    sub_option);
}

BT::NodeStatus IsChargingCondition::tick()
{
  callback_group_executor_.spin_some();
  if (is_charging_) {
    rclcpp::Duration charging_time = steady_clock_.now() - start_time_;
    RCLCPP_INFO(node_->get_logger(), "################################");
    RCLCPP_INFO(node_->get_logger(), "charging_time: %f", charging_time);
    RCLCPP_INFO(node_->get_logger(), "start_time_: %f", start_time_);
    RCLCPP_INFO(node_->get_logger(), "is_charging_: %d", is_charging_);



    if(charging_time.seconds() > charing_duration_)
      return BT::NodeStatus::SUCCESS;
    else
      return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

void IsChargingCondition::chargingCallback(std_msgs::msg::Int8::SharedPtr msg)
{

  if (msg->data == 1) {
    if(!is_charging_) start_time_ = steady_clock_.now();
    is_charging_ = true;
  } else {
    is_charging_ = false;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsChargingCondition>("IsCharging");
}
