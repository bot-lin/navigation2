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

#include "nav2_waypoint_follower/plugins/spin_wp.hpp"

#include <string>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

#include "nav2_util/node_utils.hpp"
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/point.hpp>

using json = nlohmann::json;

namespace nav2_waypoint_follower
{
SpinWp::SpinWp()
: is_enabled_(true)
{
}

SpinWp::~SpinWp()
{
}

void SpinWp::initialize(
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

  time_allowance_ = j["time_allowance"];
  target_ = j["target"];
  this->spin_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(
      node,
      "spin");

  if (!is_enabled_) {
      RCLCPP_INFO(
        logger_, "Waypoint task executor plugin is disabled.");
    }
  is_done_ = false;
}

bool SpinWp::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }
  RCLCPP_INFO(
    logger_, "Arrived at %i'th waypoint",
    curr_waypoint_index);
  if (!this->spin_client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(logger_, "Action server not available after waiting");
    rclcpp::shutdown(); 
  }
  auto goal_msg = nav2_msgs::action::Spin::Goal();
  goal_msg.time_allowance = rclcpp::Duration::from_seconds(time_allowance_);
  goal_msg.target_yaw = target_;
  auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
  send_goal_options.result_callback =
      std::bind(&SpinWp::result_callback, this, std::placeholders::_1);
  this->spin_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  while(!is_done_){
    usleep(1000);
  }
  return true;
}

void SpinWp::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult & result)
{

  is_done_ = true;
  switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(logger_, "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(logger_, "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(logger_, "Unknown result code");
        return;
    }
}
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::SpinWp,
  nav2_core::WaypointTaskExecutor)
