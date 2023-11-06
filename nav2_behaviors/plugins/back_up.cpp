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

#include "back_up.hpp"

namespace nav2_behaviors
{

Status BackUp::change_goal(const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Backing up in Y and Z not supported, will only move in X.");
    return Status::FAILED;
  }

  // Silently ensure that both the speed and direction are negative.
  command_x_ = -std::fabs(command->target.x);
  command_speed_ = -std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  end_time_ = steady_clock_.now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

Status BackUp::onRun(const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_INFO(
      logger_,
      "Backing up in Y and Z not supported, will only move in X.");
    return Status::FAILED;
  }
 
  // Silently ensure that both the speed and direction are negative.
  command_x_ = std::fabs(command->target.x);
  command_speed_ = std::fabs(command->speed);
  command_time_allowance_ = command->time_allowance;

  acc_ = command->acc;
  dec_ = command->dec;

  D_cruise_ = -1.0;
  while (D_cruise_ < 0) {
    command_speed_ = command_speed_ - 0.02;
    if (command_speed_ < 0) {
      RCLCPP_ERROR(this->logger_, "D_cruise_ is negative");
      return Status::FAILED;
    }
    D_acc_ = command_speed_ * command_speed_ / (2 * acc_);
    D_dec_ = command_speed_ * command_speed_ / (2 * dec_);
    D_cruise_ = command_x_ - (D_acc_ + D_dec_);
  }
  sign_ = -1;

  end_time_ = steady_clock_.now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
      initial_pose_, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return Status::FAILED;
  }

  return Status::SUCCEEDED;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::BackUp, nav2_core::Behavior)
