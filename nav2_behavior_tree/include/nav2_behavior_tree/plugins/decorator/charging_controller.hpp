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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__CHARGING_CONTROLLER_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__CHARGING_CONTROLLER_HPP_

#include <chrono>
#include <string>
#include "std_msgs/msg/int8.hpp"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks if charging
 */
class ChargingController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::ChargingController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ChargingController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("charging_topic", std::string("/charging_status"), "Charging topic")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  void chargingCallback(std_msgs::msg::Int8::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr charging_sub_;
  std::string charging_topic_;
  bool is_charging_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__CHARGING_CONTROLLER_HPP_
