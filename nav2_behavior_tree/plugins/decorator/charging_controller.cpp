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

#include <chrono>
#include <string>

#include "nav2_behavior_tree/plugins/decorator/charging_controller.hpp"

namespace nav2_behavior_tree
{

ChargingController::ChargingController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  charging_topic_("/charging_status"),
  is_charging_(false),
  first_time_(true)
{
  getInput("charging_topic", charging_topic_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Topic  %s", charging_topic_.c_str());

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  charging_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
    charging_topic_,
    10,
    std::bind(&ChargingController::chargingCallback, this, std::placeholders::_1),
    sub_option);
}



BT::NodeStatus ChargingController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    first_time_ = true;
  }
  setStatus(BT::NodeStatus::RUNNING);
  callback_group_executor_.spin_some();
  // The child gets ticked the first time through and any time the period has
  // expired. In addition, once the child begins to run, it is ticked each time
  // 'til completion

  if (first_time_)
  {
    sleep(1);
    first_time_ = false;
  } 
  if ((child_node_->status() == BT::NodeStatus::RUNNING) ||
    is_charging_)
  {
    const BT::NodeStatus child_state = child_node_->executeTick();
    switch (child_state) {
      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      case BT::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;

      case BT::NodeStatus::FAILURE:
      default:
        return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    return BT::NodeStatus::SUCCESS;
  }

  return status();
}

void ChargingController::chargingCallback(std_msgs::msg::Int8::SharedPtr msg)
{

  if (msg->data == 1) {
    is_charging_ = true;
  } else {
    is_charging_ = false;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::ChargingController>("ChargingController");
}
