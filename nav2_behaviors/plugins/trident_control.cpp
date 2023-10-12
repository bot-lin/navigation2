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

#include "trident_control.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_behaviors
{
TridentControl::TridentControl()
: TimedBehavior<ActuatorControlAction>(),
  feedback_(std::make_shared<ActuatorControlAction::Feedback>())
{}

void TridentControl::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  di_status_sub_ = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
  "io_module/di_status", rclcpp::SystemDefaultsQoS(),
  std::bind(
    &TridentControl::diStatusCallback,
    this, std::placeholders::_1));
  
    do_client_ = node->create_client<zbot_interfaces::srv::DigitalOutputControlSrv>("digital_output_control");


  actuator_command_pub_ = node->create_publisher<std_msgs::msg::UInt8MultiArray>("io_module/di_status", 10);
}

Status TridentControl::change_goal(const std::shared_ptr<const ActuatorControlAction::Goal> command)
{
  preempt_teleop_ = false;
  task_id_ = command->task_index;
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;
  return Status::SUCCEEDED;
}

Status TridentControl::onRun(const std::shared_ptr<const ActuatorControlAction::Goal> command)
{
  preempt_teleop_ = false;
  task_id_ = command->task_index
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;
  return Status::SUCCEEDED;

}

void TridentControl::onActionCompletion()
{
  preempt_teleop_ = false;
  arm_left_status_ = 0;
  arm_right_status_ = 0;
  arm_middle_status_ = 0;
}

Status TridentControl::onCycleUpdate()
{
  feedback_->actuator_status = actuator_status_;
  action_server_->publish_feedback(feedback_);

  rclcpp::Duration time_remaining = end_time_ - steady_clock_.now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN_STREAM(
      logger_,
      "Exceeded time allowance before reaching the " << behavior_name_.c_str() <<
        "goal - Exiting " << behavior_name_.c_str());
    // auto message = std_msgs::msg::Int32();
    // message.data = 0;
    // actuator_command_pub_->publish(message);
    return Status::FAILED;
  }

  // Actuator done successfully
  if (preempt_teleop_) {
    // auto message = std_msgs::msg::Int32();
    // message.data = 0;
    // actuator_command_pub_->publish(message);
    return Status::SUCCEEDED;
  }
  if (task_id_ == 1)
  {
    // stretch all arm
  }
  else if (task_id_ == 2)
  {
    // retract all arm
  }
  

  switch (actuator_status_)
  {
  case 0:
  {
      // auto message = std_msgs::msg::Int32();
      // message.data = current_task_index_;
      // actuator_command_pub_->publish(message);
      return Status::RUNNING;
  }
  case 1:
  {
    if (task_id_ == 1)
    {
      return Status::SUCCEEDED;
    }
    else
    {
      return Status::RUNNING;
    }

  }
  case 2:
  {
    if (task_id_ == 2)
    {
      return Status::SUCCEEDED;
    }
    else
    {
      return Status::RUNNING;
    }
  }
  case 4:
  {
    return Status::FAILED;
  }
  
  default:
    return Status::RUNNING;
  }

  
  
}


void TridentControl::preemptActuatorCallback(const std_msgs::msg::Empty::SharedPtr)
{
  preempt_teleop_ = true;
}

void TridentControl::diStatusCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{

  if (msg->data[0])
  {
    arm_left_status_ = 1; // 1 is streched
    callDOService(1, false);
    //pub command to stop left arm stretch
  }
  else if (msg->data[1])
  {
    arm_left_status_ = 2; // 2 is retracted
    //pub command to stop left arm retract
    callDOService(2, false);

  }
  else
  {
    arm_left_status_ = 0;
  }

  if (msg->data[2])
  {
    arm_right_status_ = 1; // 1 is streched
    //pub command to stop right arm stretch
  }
  else if (msg->data[3])
  {
    arm_right_status_ = 2; // 2 is retracted
    //pub command to stop right arm retract
  }
  else
  {
    arm_right_status_ = 0;
  }

  if (msg->data[4])
  {
    arm_middle_status_ = 1; // 1 is streched
    //pub command to stop middle arm stretch
  }
  else if (msg->data[5])
  {
    arm_middle_status_ = 2; // 2 is retracted
    //pub command to stop middle arm retract
  }
  else
  {
    arm_middle_status_ = 0;
  }

  if (arm_left_status_ == 1 && arm_right_status_ == 1 && arm_middle_status_ == 1) // all arms are streched
  {
    actuator_status_ = 1;
  }
  else if (arm_left_status_ == 2 && arm_right_status_ == 2 && arm_middle_status_ == 2) // all arms are retracted
  {
    actuator_status_ = 2;
  }
  else
  {
    actuator_status_ = 0;
  }

  actuator_status_ = msg->data;
  RCLCPP_INFO(
      logger_,
      "arm status: %d %d %d %d", arm_left_status_, arm_right_status_, arm_middle_status_, actuator_status_);

}

void TridentControl::callDOService(int index, bool value)
{
  auto request = std::make_shared<zbot_interfaces::srv::DigitalOutputControlSrv::Request>();
  request->id = index;
  request->command = value;
  auto result = do_client_->async_send_request(request);
  RCLCPP_INFO(
      logger_,
      "call DO service: %d %d", index, value);

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::TridentControl, nav2_core::Behavior)
