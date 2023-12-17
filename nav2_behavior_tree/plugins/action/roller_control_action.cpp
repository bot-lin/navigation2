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

#include "nav2_behavior_tree/plugins/action/roller_control_action.hpp"

namespace nav2_behavior_tree
{

RollerControlAction::RollerControlAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<zbot_interfaces::action::CustomProgramAction>(xml_tag_name, action_name, conf)
{


}

void RollerControlAction::on_tick()
{
    int task_id;
    std::string task_name;
    
  getInput("task_id", task_id);
  getInput("task_name", task_name);
  getInput("task_param", goal_.params);
  goal_.task_id = task_id;
  goal_.task_name = task_name;
}

BT::NodeStatus RollerControlAction::on_success()
{
  setOutput("pose", result_.pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::RollerControlAction>(name, "custom_program_action", config);
    };

  factory.registerBuilder<nav2_behavior_tree::RollerControlAction>("RollerControl", builder);
}
