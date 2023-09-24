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

#include <string>
#include <memory>

#include "std_msgs/msg/string.hpp"

#include "nav2_behavior_tree/plugins/action/find_cylinder_centroid_action.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{

using std::placeholders::_1;

FindCylinderCentroid::FindCylinderCentroid(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  
  client_ =
    node_->create_client<zbot_interfaces::srv::FindCylinderSrv>("find_cylinder_centroid");

 
}

BT::NodeStatus FindCylinderCentroid::tick()
{
  auto request = std::make_shared<zbot_interfaces::srv::FindCylinderSrv::Request>();
  getInput("radius_limits", request->radius_limits);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Centroid: %f,%f,%f", result.get()->centroid.pose.position.x, result.get()->centroid.pose.position.y, result.get()->centroid.pose.position.z);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  return BT::NodeStatus::SUCCESS;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::FindCylinderCentroid>("FindCylinderCentroid");
}
