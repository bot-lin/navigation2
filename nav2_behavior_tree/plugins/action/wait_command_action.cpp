// Copyright (c) 2022 Joshua Wallace
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

#include "nav2_behavior_tree/plugins/action/wait_command_action.hpp"

namespace nav2_behavior_tree
{

WaitCommandAction::WaitCommandAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::WaitCommand>(xml_tag_name, action_name, conf)
{
  double time_allowance, vel_x, vel_z;
  getInput("time_allowance", time_allowance);
  getInput("vel_x", vel_x);
  getInput("vel_z", vel_z);

  // Populate the input message
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
  goal_.vel_x = vel_x;
  goal_.vel_z = vel_z;
}

void WaitCommandAction::on_tick()
{
  increment_recovery_count();
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::WaitCommandAction>(
        name, "wait_command", config);
    };

  factory.registerBuilder<nav2_behavior_tree::WaitCommandAction>("WaitCommand", builder);
}
