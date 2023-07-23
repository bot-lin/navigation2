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


  vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_teleop, rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ActuatorControl::teleopVelocityCallback,
      this, std::placeholders::_1));

  preempt_teleop_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    "preempt_teleop", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &ActuatorControl::preemptTeleopCallback,
      this, std::placeholders::_1));
}

Status ActuatorControl::change_goal(const std::shared_ptr<const ActuatorControlAction::Goal> command)
{
  std::string actuator_index = command->actuator_index;
  actuator_command_pub_ = node->create_publisher<std_msgs::msg::String>("topic", 10);
  auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
  actuator_command_pub_->publish(message)
  actuator_command_pub_.reset();
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;
  return Status::SUCCEEDED;
}

Status ActuatorControl::onRun(const std::shared_ptr<const ActuatorControlAction::Goal> command)
{
  command_time_allowance_ = command->time_allowance;
  end_time_ = steady_clock_.now() + command_time_allowance_;
  return Status::SUCCEEDED;
}

void ActuatorControl::onActionCompletion()
{
}

Status ActuatorControl::onCycleUpdate()
{
  feedback_->current_teleop_duration = elasped_time_;
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
    stopRobot();
    return Status::SUCCEEDED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_base_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Current robot pose is not available for " <<
        behavior_name_.c_str());
    return Status::FAILED;
  }
  geometry_msgs::msg::Pose2D projected_pose;
  projected_pose.x = current_pose.pose.position.x;
  projected_pose.y = current_pose.pose.position.y;
  projected_pose.theta = tf2::getYaw(current_pose.pose.orientation);

  for (double time = simulation_time_step_; time < projection_time_;
    time += simulation_time_step_)
  {
    if (!collision_checker_->isCollisionFree(projected_pose)) {
      if (time == simulation_time_step_) {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          logger_,
          steady_clock_,
          1000,
          behavior_name_.c_str() << " collided on first time step, setting velocity to zero");
        scaled_twist.linear.x = 0.0f;
        scaled_twist.linear.y = 0.0f;
        scaled_twist.angular.z = 0.0f;
        break;
      } else {
        RCLCPP_DEBUG_STREAM_THROTTLE(
          logger_,
          steady_clock_,
          1000,
          behavior_name_.c_str() << " collision approaching in " << time << " seconds");
        double scale_factor = time / projection_time_;
        scaled_twist.linear.x *= scale_factor;
        scaled_twist.linear.y *= scale_factor;
        scaled_twist.angular.z *= scale_factor;
        break;
      }
    }
  }
  vel_pub_->publish(std::move(scaled_twist));

  return Status::RUNNING;
}



void ActuatorControl::teleopVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  teleop_twist_ = *msg;
}

void ActuatorControl::preemptTeleopCallback(const std_msgs::msg::Empty::SharedPtr)
{
  preempt_teleop_ = true;
}

}  // namespace nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::ActuatorControl, nav2_core::Behavior)
