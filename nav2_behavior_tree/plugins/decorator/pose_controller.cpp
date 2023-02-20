// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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
#include <memory>
#include <cmath>

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

#include "behaviortree_cpp_v3/decorator_node.h"

#include "nav2_behavior_tree/plugins/decorator/pose_controller.hpp"

namespace nav2_behavior_tree
{

PoseController::PoseController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  distance_(0.1),
  yaw_distance_(1.0),
  global_frame_("map"),
  robot_base_frame_("base_link"),
  first_time_(false),
  target_x_(0.0),
  target_y_(0.0),
  target_yaw_(0.0)
{
  getInput("distance", distance_);
  getInput("yaw_distance", yaw_distance_);
  getInput("global_frame", global_frame_);
  getInput("robot_base_frame", robot_base_frame_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  tf2::Quaternion tf2_quat;
  tf2_quat.setRPY(0.0, 0.0, target_yaw_);
  target_pose_.pose.position.x=  target_x_;
  target_pose_.pose.position.y=  target_y_;
  target_pose_.pose.orientation = tf2::toMsg(tf2_quat);

  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

inline BT::NodeStatus PoseController::tick()
{

  setStatus(BT::NodeStatus::RUNNING);

  // Determine distance travelled since we've started this iteration
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Get euclidean distance
  auto remaining = nav2_util::geometry_utils::euclidean_distance(
    target_pose_.pose, current_pose.pose);
  
  tf2::Quaternion q(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
  
  double yaw_remained = abs(yaw-target_yaw_);
  bool in_the_zone = false;
  if (yaw_remained < yaw_distance_ && remaining < distance_) in_the_zone = true;

  // The child gets ticked the first time through and every time the threshold
  // distance is crossed. In addition, once the child begins to run, it is
  // ticked each time 'til completion
  if ((child_node_->status() == BT::NodeStatus::RUNNING) ||
    in_the_zone)
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

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PoseController>("PoseController");
}
