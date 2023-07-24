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

#include "actuator_control.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{
ActuatorControl::ActuatorControl()
: TimedBehavior<ActuatorControlAction>(),
  feedback_(std::make_shared<ActuatorControlAction::Feedback>())
{}

void ActuatorControl::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }


  // preempt_actuator_sub_ = node->create_subscription<std_msgs::msg::Empty>(
  //   "stop_actuator_command", rclcpp::SystemDefaultsQoS(),
  //   std::bind(
  //     &ActuatorControl::preemptActuatorCallback,
  //     this, std::placeholders::_1));

  // preempt_actuator_sub_ = node->create_subscription<std_msgs::msg::Empty>(
  //   "stop_actuator_command", rclcpp::SystemDefaultsQoS(),
  //   std::bind(
  //     &ActuatorControl::preemptActuatorCallback,
  //     this, std::placeholders::_1));
}

Status ActuatorControl::change_goal(const std::shared_ptr<const ActuatorControlAction::Goal> command)
{
    auto node = node_.lock();
    std::string actuator_index = command->actuator_index;
    actuator_command_pub_ = node->create_publisher<std_msgs::msg::Int32>("actuator_command" + actuator_index, 10);
    auto message = std_msgs::msg::Int32();
        message.data = command->task_index;
    actuator_command_pub_->publish(message);
    actuator_command_pub_.reset();
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;
  return Status::SUCCEEDED;
}

Status ActuatorControl::onRun(const std::shared_ptr<const ActuatorControlAction::Goal> command)
{
  auto node = node_.lock();
  std::string actuator_index = command->actuator_index;
  actuator_command_pub_.reset();
  actuator_command_pub_ = node->create_publisher<std_msgs::msg::Int32>("actuator_command/" + actuator_index, 10);
  auto message = std_msgs::msg::Int32();
      message.data = command->task_index;
  actuator_command_pub_->publish(message);
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;
  return Status::SUCCEEDED;
}

void ActuatorControl::onActionCompletion()
{
}

Status ActuatorControl::onCycleUpdate()
{
  feedback_->actuator_status = 1;
  action_server_->publish_feedback(feedback_);

  rclcpp::Duration time_remaining = end_time_ - steady_clock_.now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN_STREAM(
      logger_,
      "Exceeded time allowance before reaching the " << behavior_name_.c_str() <<
        "goal - Exiting " << behavior_name_.c_str());
    return Status::FAILED;
  }

  // user states that teleop was successful
  if (preempt_teleop_) {
    //
    //  stop_actuator();
    //
    return Status::SUCCEEDED;
  }

  
  return Status::RUNNING;
}


void ActuatorControl::preemptActuatorCallback(const std_msgs::msg::Empty::SharedPtr)
{
  preempt_teleop_ = true;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::ActuatorControl, nav2_core::Behavior)
