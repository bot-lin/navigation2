#include <cmath>
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "precise_nav.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
using namespace std::chrono_literals;

namespace nav2_behaviors
{

PreciseNav::PreciseNav()
: TimedBehavior<PreciseNavAction>(),
    feedback_(std::make_shared<PreciseNavAction::Feedback>()),
    target_x_(0.0),
    target_y_(0.0),
    target_yaw_(0.0)
{
}

PreciseNav::~PreciseNav() = default;

void PreciseNav::onConfigure()
{
    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error{"Failed to lock node"};
    }
    nav2_util::declare_parameter_if_not_declared(
        node,
        "distance_goal_tolerance", rclcpp::ParameterValue(0.03));
    node->get_parameter("distance_goal_tolerance", distance_goal_tolerance_);

    nav2_util::declare_parameter_if_not_declared(
        node,
        "heading_tolerance", rclcpp::ParameterValue(0.1));
    node->get_parameter("heading_tolerance", heading_tolerance_);

    nav2_util::declare_parameter_if_not_declared(
        node,
        "yaw_goal_tolerance", rclcpp::ParameterValue(0.05));
    node->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);

    nav2_util::declare_parameter_if_not_declared(
        node,
        "angular_velocity", rclcpp::ParameterValue(0.2));
    node->get_parameter("angular_velocity", angular_velocity_);

    nav2_util::declare_parameter_if_not_declared(
        node,
        "linear_velocity", rclcpp::ParameterValue(0.04));
    node->get_parameter("linear_velocity", linear_velocity_);
    RCLCPP_INFO(this->logger_, "******************* %f", yaw_goal_tolerance_);


}

Status PreciseNav::onRun(const std::shared_ptr<const PreciseNavAction::Goal> command) 
{
    geometry_msgs::msg::PoseStamped pose_tmp;
    pose_tmp.pose.position.x = command->pose.pose.position.x;
    pose_tmp.pose.position.y = command->pose.pose.position.y;
    pose_tmp.pose.orientation.x = command->pose.pose.orientation.x;
    pose_tmp.pose.orientation.y = command->pose.pose.orientation.y;
    pose_tmp.pose.orientation.z = command->pose.pose.orientation.z;
    pose_tmp.pose.orientation.w = command->pose.pose.orientation.w;
    pose_tmp.header.frame_id = command->pose.header.frame_id;
    is_reverse_ = command->is_reverse;
    RCLCPP_INFO(this->logger_, "Failed to transform goal pose in odom frame from %d", is_reverse_);

    if (command->pose.header.frame_id != "odom")
    {
        bool tf_response = nav2_util::transformPoseInTargetFrame(pose_tmp, pose_tmp,  *this->tf_, "odom", this->transform_tolerance_);
        if (!tf_response)
        {
            RCLCPP_ERROR(this->logger_, "Failed to transform goal pose in odom frame from %s", command->pose.header.frame_id.c_str());
            return Status::FAILED;
        }
        RCLCPP_INFO(this->logger_, "Converting goal pose in odom frame from %s", command->pose.header.frame_id.c_str());
        RCLCPP_INFO(this->logger_, "Converted pose x: %f, y: %f", pose_tmp.pose.position.x, pose_tmp.pose.position.y);
    }
    // else{
    //     pose_tmp = command->pose;
    // }
    target_x_ = pose_tmp.pose.position.x;
    target_y_ = pose_tmp.pose.position.y;
    tf2::Quaternion q(pose_tmp.pose.orientation.x, 
                    pose_tmp.pose.orientation.y, 
                    pose_tmp.pose.orientation.z,
                    pose_tmp.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    target_yaw_ = yaw;
    return Status::SUCCEEDED;
}

Status PreciseNav::change_goal(const std::shared_ptr<const PreciseNavAction::Goal> command)
{
    
    geometry_msgs::msg::PoseStamped pose_tmp;
    pose_tmp.pose.position.x = command->pose.pose.position.x;
    pose_tmp.pose.position.y = command->pose.pose.position.y;
    pose_tmp.pose.orientation.x = command->pose.pose.orientation.x;
    pose_tmp.pose.orientation.y = command->pose.pose.orientation.y;
    pose_tmp.pose.orientation.z = command->pose.pose.orientation.z;
    pose_tmp.pose.orientation.w = command->pose.pose.orientation.w;
    pose_tmp.header.frame_id = command->pose.header.frame_id;
    is_reverse_ = command->is_reverse;

    if (command->pose.header.frame_id != "odom")
    {
        bool tf_response = nav2_util::transformPoseInTargetFrame(pose_tmp, pose_tmp,  *this->tf_, "odom", this->transform_tolerance_);
        if (!tf_response)
        {
            RCLCPP_ERROR(this->logger_, "Failed to transform goal pose in odom frame from %s", command->pose.header.frame_id.c_str());
            return Status::FAILED;
        }
        RCLCPP_INFO(this->logger_, "Converting goal pose in odom frame from %s", command->pose.header.frame_id.c_str());
        
    }
    else{
        pose_tmp = command->pose;
    }
    RCLCPP_INFO(this->logger_, "target pose in Odom x: %f, y: %f", pose_tmp.pose.position.x, pose_tmp.pose.position.y);
    target_x_ = pose_tmp.pose.position.x;
    target_y_ = pose_tmp.pose.position.y;
    tf2::Quaternion q(pose_tmp.pose.orientation.x, 
                    pose_tmp.pose.orientation.y, 
                    pose_tmp.pose.orientation.z,
                    pose_tmp.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    target_yaw_ = yaw;
    return Status::SUCCEEDED;
}
Status PreciseNav::onCycleUpdate()
{
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *this->tf_, "odom", this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }
    double distance_to_goal = getDistanceToGoal(current_pose);
    double heading_error = getHeadingErrorToGoal(current_pose);
    double yaw_goal_error = getRadiansToGoal(current_pose);

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    if (distance_to_goal > distance_goal_tolerance_ && !reached_distance_goal_)
    {
        if (std::fabs(heading_error) > heading_tolerance_){
            cmd_vel->linear.x = 0.005;
            if (heading_error > 0) cmd_vel->angular.z = angular_velocity_;
            else cmd_vel->angular.z = -angular_velocity_;
        }
        else
        {
            if (is_reverse_) cmd_vel->linear.x = -linear_velocity_;
            else cmd_vel->linear.x = linear_velocity_;
        }
    }
    else if (std::fabs(yaw_goal_error) > yaw_goal_tolerance_)
    {
        cmd_vel->angular.z = 0.3 * yaw_goal_error;
        reached_distance_goal_ = true;
    }
    else
    {
        reached_distance_goal_ = false;
        this->stopRobot();
        return Status::SUCCEEDED;
    }
    RCLCPP_INFO(this->logger_, "current pose x: %f, y: %f", current_pose.pose.position.x, current_pose.pose.position.y);
    RCLCPP_INFO(this->logger_, "current pose z: %f, w: %f", current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    RCLCPP_INFO(this->logger_, "pub vel x: %f, w: %f", cmd_vel->linear.x, cmd_vel->angular.z);
    this->vel_pub_->publish(std::move(cmd_vel));
    return Status::RUNNING;
}

double PreciseNav::getDistanceToGoal(geometry_msgs::msg::PoseStamped current_pose)
{   
    double diff_x = target_x_ - current_pose.pose.position.x;
    double diff_y = target_y_ - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);
    return distance;
}

double PreciseNav::getHeadingErrorToGoal(geometry_msgs::msg::PoseStamped current_pose)
{
    tf2::Quaternion q(current_pose.pose.orientation.x, 
                        current_pose.pose.orientation.y, 
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_robot_heading;
    m.getRPY(roll, pitch, current_robot_heading);
    double delta_x, delta_y;
    if (is_reverse_){
        delta_x = current_pose.pose.position.x - target_x_ ;
        delta_y = current_pose.pose.position.y - target_y_;
    }else{
        delta_x = target_x_ - current_pose.pose.position.x;
        delta_y = target_y_ - current_pose.pose.position.y;
    }

    double desired_heading = std::atan2(delta_y, delta_x);
    double heading_error = desired_heading - current_robot_heading;
    if (heading_error > M_PI) heading_error = heading_error - (2 * M_PI);
    if (heading_error < -M_PI) heading_error = heading_error + (2 * M_PI);
    return heading_error;
}

double PreciseNav::getRadiansToGoal(geometry_msgs::msg::PoseStamped current_pose)
{
    tf2::Quaternion q(current_pose.pose.orientation.x, 
                        current_pose.pose.orientation.y, 
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_robot_heading;
    m.getRPY(roll, pitch, current_robot_heading);
    double yaw_goal_angle_error = target_yaw_ - current_robot_heading;
    if (yaw_goal_angle_error > M_PI) yaw_goal_angle_error = yaw_goal_angle_error - (2 * M_PI);
    if (yaw_goal_angle_error < -M_PI) yaw_goal_angle_error = yaw_goal_angle_error + (2 * M_PI);
    return yaw_goal_angle_error;
}


} // namespace name
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::PreciseNav, nav2_core::Behavior)

