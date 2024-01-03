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
  //getInput("goal", goal_.pose);

  double time_allowance;
  getInput("time_allowance", time_allowance);
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);


  if (!getInput("is_reverse", goal_.is_reverse)) {
    goal_.is_reverse = false;
  }
  if (!getInput("reverse_yaw", goal_.reverse_yaw)) {
    goal_.reverse_yaw = false;
  }
  if (!getInput("distance_goal_tolerance", goal_.distance_goal_tolerance)) {
    goal_.distance_goal_tolerance = 0.03;
  }
  if (!getInput("yaw_goal_tolerance", goal_.yaw_goal_tolerance)) {
    goal_.yaw_goal_tolerance = 0.05;
  }
  if (!getInput("target_tf_frame", goal_.target_tf_frame)) {
    goal_.target_tf_frame = "odom";
  }
  if (!getInput("orientation_p", goal_.orientation_p)) {
    goal_.orientation_p = 1.0;
  }
  if (!getInput("orientation_i", goal_.orientation_i)) {
    goal_.orientation_i = 0.05;
  }
  if (!getInput("orientation_d", goal_.orientation_d)) {
    goal_.orientation_d = 0.1;
  }
  if (!getInput("max_linear", goal_.max_linear)) {
    goal_.max_linear = 0.5;
  }
  if (!getInput("max_angular", goal_.max_angular)) {
    goal_.max_angular = 0.5;
  }
  if (!getInput("scale_factor", goal_.scale_factor)) {
    goal_.scale_factor = 2.0;
  }
  if (!getInput("distance_max", goal_.distance_max)) {
    goal_.distance_max = 1.0;
  }
  if (!getInput("smoothing_factor", goal_.smoothing_factor)) {
    goal_.smoothing_factor = 0.7;
  }

  getInput("is_heading_only", goal_.heading_only);
}

void PreciseNavAction::on_wait_for_result(
  std::shared_ptr<const nav2_msgs::action::PreciseNav::Feedback>/*feedback*/)
{
  geometry_msgs::msg::PoseStamped new_goal;
  bool is_reverse, reverse_yaw, is_heading_only;
  float distance_goal_tolerance;
  float yaw_goal_tolerance;
  std::string target_tf_frame;
  getInput("goal", new_goal);
  getInput("is_reverse", is_reverse);
  getInput("reverse_yaw", reverse_yaw);
  getInput("distance_goal_tolerance", distance_goal_tolerance);
  getInput("yaw_goal_tolerance", yaw_goal_tolerance);
  getInput("is_heading_only", is_heading_only);
    if (!getInput("target_tf_frame", target_tf_frame)) {
    target_tf_frame = "odom";
  }

  
  if (new_goal != goal_.pose)
  {
    goal_.pose = new_goal;
    goal_.is_reverse = is_reverse;
    goal_.distance_goal_tolerance = distance_goal_tolerance;
    goal_.yaw_goal_tolerance = yaw_goal_tolerance;
    goal_.reverse_yaw = reverse_yaw;
    goal_.heading_only = is_heading_only;
    goal_.target_tf_frame = target_tf_frame;
    goal_updated_ = true;
  }
  
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
