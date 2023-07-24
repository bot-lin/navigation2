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

#ifndef NAV2_BEHAVIORS__PLUGINS__ACTUATOR_CONTROL_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ACTUATOR_CONTROL_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/actuator_control.hpp"

namespace nav2_behaviors
{
using ActuatorControlAction = nav2_msgs::action::ActuatorControl;

/**
 * @class nav2_behaviors::ActuatorControl
 * @brief An action server behavior for actuator control
 */
class ActuatorControl : public TimedBehavior<ActuatorControlAction>
{
public:
  ActuatorControl();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const ActuatorControlAction::Goal> command) override;
  Status change_goal(const std::shared_ptr<const ActuatorControlAction::Goal> command) override;

  /**
   * @brief func to run at the completion of the action
   */
  void onActionCompletion() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  Status onCycleUpdate() override;

protected:
  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;


  /**
   * @brief Callback function to preempt actuator control
   * @param msg empty message
   */
  void preemptActuatorCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void actuatorStatusCallback(const std_msgs::msg::Int32::SharedPtr msg);

  ActuatorControlAction::Feedback::SharedPtr feedback_;

  // parameters
  double projection_time_;
  double simulation_time_step_;

  bool preempt_teleop_{false};

  //publishers
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr actuator_command_pub_;
  // subscribers
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr actuator_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr preempt_actuator_sub_;
  

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
};
}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__ACTUATOR_CONTROL_HPP_
