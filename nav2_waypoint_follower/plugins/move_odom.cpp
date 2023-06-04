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

#include "nav2_waypoint_follower/plugins/move_odom.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/point.hpp>

using json = nlohmann::json;

namespace nav2_waypoint_follower
{
MoveOdom::MoveOdom()
: is_enabled_(true)
{
}

MoveOdom::~MoveOdom()
{
}

void MoveOdom::initialize(
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

  direnction_ = j["direction"];
  speed_ = j['speed'];
  target_ = j['target'];
  if (direnction_ == 0){
    if (speed_ > 0){
      //forward
    }else{
      //backward
      this->backup_client_ptr_ = rclcpp_action::create_client<BackUp>(
      this,
      "backup");
    }
  }

  if (!is_enabled_) {
      RCLCPP_INFO(
        logger_, "Waypoint task executor plugin is disabled.");
    }
  is_done_ = false;
}

bool MoveOdom::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }
  RCLCPP_INFO(
    logger_, "Arrived at %i'th waypoint, controlling fork height  to %i millimeter",
    curr_waypoint_index,
    height_control);
  if (!this->backup_client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }
  auto goal_msg = BackUp::Goal();
  auto message = geometry_msgs::msg::Point();
  message.x = target_;
  goal_msg.speed = speed_;
  goal_msg.time_allowance = 100000;
  goal_msg.target = message
  auto send_goal_options = rclcpp_action::Client<BackUp>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&MoveOdom::result_callback, this, _1);
  this->backup_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  while(!is_done_){
    usleep(1000);
  }
  return true;
}

void MoveOdom::result_callback(const rclcpp_action::ClientGoalHandle<BackUp>::WrappedResult & result)
{
  is_done_ = true;
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::MoveOdom,
  nav2_core::WaypointTaskExecutor)
