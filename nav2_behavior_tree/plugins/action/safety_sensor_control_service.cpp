// Copyright (c) 2019 Samsung Research America
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
#include "nav2_behavior_tree/plugins/action/safety_sensor_control_service.hpp"

namespace nav2_behavior_tree
{

SafetySensorControlService::SafetySensorControlService(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: BtServiceNode<std_srvs::srv::SetBool>(service_node_name, conf)
{}

void SafetySensorControlService::on_tick()
{
  getInput("bool_value", request_->data);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SafetySensorControlService>(
    "SafetySensorControl");
}
