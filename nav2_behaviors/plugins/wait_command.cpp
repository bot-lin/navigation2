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

#include <utility>

#include "wait_command.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{
WaitCommand::WaitCommand()
: TimedBehavior<WaitCommandAction>(),
  feedback_(std::make_shared<WaitCommandAction::Feedback>())
{}

void WaitCommand::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  preempt_wait_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    "wait_util_command", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &WaitCommand::preemptTeleopCallback,
      this, std::placeholders::_1));
}

Status WaitCommand::change_goal(const std::shared_ptr<const WaitCommandAction::Goal> command)
{
  preempt_wait_ = false;
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;
  return Status::SUCCEEDED;
}

Status WaitCommand::onRun(const std::shared_ptr<const WaitCommandAction::Goal> command)
{
  preempt_wait_ = false;
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;
  return Status::SUCCEEDED;
}

void WaitCommand::onActionCompletion()
{
  preempt_wait_ = false;
}

Status WaitCommand::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - steady_clock_.now();
  feedback_->time_left = time_remaining;
  action_server_->publish_feedback(feedback_);
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN_STREAM(
      logger_,
      "Exceeded time allowance before reaching the " << behavior_name_.c_str() <<
        "goal - Exiting " << behavior_name_.c_str());
    return Status::SUCCEEDED;
  }

  // user states that teleop was successful
  if (preempt_wait_) {
    stopRobot();
    return Status::SUCCEEDED;
  }
  return Status::RUNNING;
}


void WaitCommand::preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr)
{
  preempt_wait_ = true;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::WaitCommand, nav2_core::Behavior)
