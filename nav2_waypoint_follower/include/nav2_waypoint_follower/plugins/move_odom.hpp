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

#ifndef NAV2_WAYPOINT_FOLLOWER__PLUGINS__MOVE_ODOM_HPP_
#define NAV2_WAYPOINT_FOLLOWER__PLUGINS__MOVE_ODOM_HPP_
#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include <std_msgs/msg/u_int64_multi_array.hpp>


namespace nav2_waypoint_follower
{

/**
 * @brief Simple plugin based on WaypointTaskExecutor, lets robot to sleep for a
 *        specified amount of time at waypoint arrival. You can reference this class to define
 *        your own task and rewrite the body for it.
 *
 */
class MoveOdom : public nav2_core::WaypointTaskExecutor
{
public:
/**
 * @brief Construct a new Wait At Waypoint Arrival object
 *
 */
  MoveOdom();

  /**
   * @brief Destroy the Wait At Waypoint Arrival object
   *
   */
  ~MoveOdom();

  /**
   * @brief declares and loads parameters used (waypoint_pause_duration_)
   *
   * @param parent parent node that plugin will be created withing(waypoint_follower in this case)
   * @param plugin_name
   */
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name, const std::string & params);


  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * @return false if task execution failed
   */
  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index);
  void result_callback(const rclcpp_action::ClientGoalHandle<BackUp>::WrappedResult & result)
protected:
  // the robot will sleep waypoint_pause_duration_ milliseconds
  int direnction_; //linear 0; angular 1; 
  float speed_; 
  float target_;
  bool is_enabled_;
  bool is_done_;
  rclcpp::Logger logger_{rclcpp::get_logger("nav2_waypoint_follower")};
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_ptr_;
};

}  // namespace nav2_waypoint_follower
#endif  // NAV2_WAYPOINT_FOLLOWER__PLUGINS__MOVE_ODOM_HPP_
