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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRECISE_NAV_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRECISE_NAV_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/precise_nav.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::PreciseNav
 */
class PreciseNavAction : public BtActionNode<nav2_msgs::action::PreciseNav>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PreciseNavAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for 
   * @param conf BT node configuration
   */
  PreciseNavAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);
  void on_tick() override;
  void on_wait_for_result(
    std::shared_ptr<const nav2_msgs::action::PreciseNav::Feedback> feedback) override;

  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;
  void halt() override;
  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<bool>("is_reverse", "Is reverse"),
        BT::InputPort<bool>("reverse_yaw", "Reverse yaw"),
        BT::InputPort<bool>("is_heading_only", "is_heading_only"),
        BT::InputPort<float>("distance_goal_tolerance", "distance goal tolerance"),
        BT::InputPort<float>("yaw_goal_tolerance", "yaw goal tolerance"),
        BT::InputPort<double>("orientation_p", "orientation p"),
        BT::InputPort<double>("orientation_i", "orientation i"),
        BT::InputPort<double>("orientation_d", "orientation d"),
        BT::InputPort<double>("max_linear", "max_linear"),
        BT::InputPort<double>("max_angular", "max_angular"),
        BT::InputPort<double>("scale_factor", "scale_factor"),
        BT::InputPort<double>("distance_max", "distance_max"),
        BT::InputPort<double>("smoothing_factor", "smoothing_factor"),

        BT::InputPort<std::string>("target_tf_frame", "The target frame for the goal pose"),
      });
  }
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PRECISE_NAV_ACTION_HPP_
