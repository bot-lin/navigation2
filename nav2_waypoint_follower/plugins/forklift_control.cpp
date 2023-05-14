// Copyright (c) 2020 Fetullah Atas
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

#include "nav2_waypoint_follower/plugins/forklift_control.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace nav2_waypoint_follower
{
ForkliftControl::ForkliftControl()
: is_enabled_(true)
{
}

ForkliftControl::~ForkliftControl()
{
}

void ForkliftControl::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name, const std::string & params)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in wait at waypoint plugin!"};
  }
  RCLCPP_INFO(
      logger_,
      "%s Task params %s", plugin_name.c_str(), params.c_str());
  json j = json::parse(params);
  logger_ = node->get_logger();

  fork_height_publisher_ = node->create_publisher<std_msgs::msg::UInt64MultiArray>("fk_height", 10);


  height_control = j["control_height"];
  if (height_control < 0) {
    is_enabled_ = false;
    RCLCPP_INFO(
      logger_,
      "Fork control cannot be set to lower than 0, disabling task executor plugin.");
  } else if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Waypoint task executor plugin is disabled.");
  }
}

bool ForkliftControl::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }
  RCLCPP_INFO(
    logger_, "Arrived at %i'th waypoint, controlling fork height  to %i millimeter",
    curr_waypoint_index,
    height_control);
  auto message = std_msgs::msg::UInt64MultiArray();
  message.data.push_back(0);
  message.data.push_back(height_control);
  fork_height_publisher_->publish(message);
  return true;
}
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::ForkliftControl,
  nav2_core::WaypointTaskExecutor)
