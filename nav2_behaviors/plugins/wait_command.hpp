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

#ifndef NAV2_BEHAVIORS__PLUGINS__WAIT_COMMAND_HPP_
#define NAV2_BEHAVIORS__PLUGINS__WAIT_COMMAND_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/wait_command.hpp"

namespace nav2_behaviors
{
using WaitCommandAction = nav2_msgs::action::WaitCommand;

/**
 * @class nav2_behaviors::WaitCommand
 * @brief An action server behavior for wait command
 */
class WaitCommand : public TimedBehavior<WaitCommandAction>
{
public:
  WaitCommand();

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const WaitCommandAction::Goal> command) override;
  Status change_goal(const std::shared_ptr<const WaitCommandAction::Goal> command) override;

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
   * @brief Callback function to preempt wait command
   * @param msg empty message
   */
  void preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr msg);

  WaitCommandAction::Feedback::SharedPtr feedback_;
  bool preempt_wait_{false};

  // subscribers
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr preempt_wait_sub_;

  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  double vel_x_{0.0};
  double vel_z_{0.0};
};
}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__WAIT_COMMAND_HPP_
