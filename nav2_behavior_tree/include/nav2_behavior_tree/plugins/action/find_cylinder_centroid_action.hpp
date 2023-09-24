// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Pablo Iñigo Blasco
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_CYLINDER_CENTROID_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__FIND_CYLINDER_CENTROID_ACTION_HPP_

#include <memory>
#include <string>

#include "zbots_msgs/srv/find_cylinder_srv.hpp"

#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief The PlannerSelector behavior is used to switch the planner
 * that will be used by the planner server. It subscribes to a topic "planner_selector"
 * to get the decision about what planner must be used. It is usually used before of
 * the ComputePathToPoseAction. The selected_planner output port is passed to planner_id
 * input port of the ComputePathToPoseAction
 */
class FindCylinderCentroid : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PlannerSelector
   *
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf  BT node configuration
   */
  FindCylinderCentroid(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>(
        "radius_limits",
        "the value of the radius limits"),

      BT::InputPort<std::string>(
        "topic_name",
        "planner_selector",
        "the input topic name to select the planner")
    };
  }

private:
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief callback function for the planner_selector topic
   *
   * @param msg the message with the id of the planner_selector
   */

  rclcpp::Client<zbot_interfaces::srv::FindCylinderSrv>::SharedPtr client_;
  rclcpp::Node::SharedPtr node_;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__PLANNER_SELECTOR_NODE_HPP_
