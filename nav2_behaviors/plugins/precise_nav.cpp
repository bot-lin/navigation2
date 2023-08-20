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
        "yaw_goal_tolerance", rclcpp::ParameterValue(0.05));
    node->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);

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
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose_tmp.pose.position.x;
    pose2d.y = pose_tmp.pose.position.y;
    pose2d.theta = tf2::getYaw(pose_tmp.pose.orientation);
    
    is_reverse_ = command->is_reverse;
    yaw_goal_tolerance_ = command->yaw_goal_tolerance;
    distance_goal_tolerance_ = command->distance_goal_tolerance;
    max_linear_velocity_ = command->max_linear_velocity;
    max_angular_velocity_ = command->max_angular_velocity;    
    position_controller_.initPID(command->position_p, command->position_i, command->position_d);
    orientation_controller_.initPID(command->orientation_p, command->orientation_i, command->orientation_d);

    RCLCPP_INFO(this->logger_, "From precise nav");
    RCLCPP_INFO(this->logger_, "Is reverse %d", is_reverse_);
    RCLCPP_INFO(this->logger_, "distance_goal_tolerance %f", distance_goal_tolerance_);
    RCLCPP_INFO(this->logger_, "yaw_goal_tolerance %f", yaw_goal_tolerance_);

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
    RCLCPP_INFO(this->logger_, "target pose in Odom x: %f, y: %f", pose_tmp.pose.position.x, pose_tmp.pose.position.y);

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
    last_pid_time_ = steady_clock_.now();
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

    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = pose_tmp.pose.position.x;
    pose2d.y = pose_tmp.pose.position.y;
    pose2d.theta = tf2::getYaw(pose_tmp.pose.orientation);
    
    is_reverse_ = command->is_reverse;
    yaw_goal_tolerance_ = command->yaw_goal_tolerance;
    distance_goal_tolerance_ = command->distance_goal_tolerance;
    max_linear_velocity_ = command->max_linear_velocity;
    max_angular_velocity_ = command->max_angular_velocity;    
    position_controller_.initPID(command->position_p, command->position_i, command->position_d);
    orientation_controller_.initPID(command->orientation_p, command->orientation_i, command->orientation_d)


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
    last_pid_time_ = steady_clock_.now();
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
    tf2::Quaternion q(current_pose.pose.orientation.x, 
                        current_pose.pose.orientation.y, 
                        current_pose.pose.orientation.z,
                        current_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, current_robot_yaw;
    m.getRPY(roll, pitch, current_robot_yaw);

    double distance_error = std::sqrt(std::pow(target_x_ - current_pose.pose.position.x, 2) + 
                                      std::pow(target_y_ - current_pose.pose.position.y, 2));
    
    double target_direction_theta = std::atan2(target_y_ - current_pose.pose.position.y, target_x_ - current_pose.pose.position.x);
    double final_orientation_error = target_yaw_ - current_robot_yaw;
    double direction_orientation_error = target_direction_theta - current_robot_yaw;

    if (is_reverse_) {
        distance_error = -distance_error;  // reverse motion
        direction_orientation_error = direction_orientation_error > 0 ? direction_orientation_error - M_PI : direction_orientation_error + M_PI;
    }
    double orientation_error = (distance_error > 0.1) ? direction_orientation_error : final_orientation_error;
    while(orientation_error > M_PI) orientation_error -= 2*M_PI;
    while(orientation_error < -M_PI) orientation_error += 2*M_PI;
    rclcpp::Time current_pid_time = steady_clock_.now();
    double dt = (current_pid_time - last_pid_time_).seconds();

    double linear_velocity = position_controller_.compute(distance_error, 0, dt);
    double angular_velocity = orientation_controller_.compute(orientation_error, 0, dt);
    last_pid_time_ = current_pid_time;

    // Clamp velocities to their respective limits
    if (std::abs(linear_velocity) > max_linear_velocity_) {
        linear_velocity = (linear_velocity > 0 ? max_linear_velocity_ : -max_linear_velocity_);
    }
    if (std::abs(angular_velocity) > max_angular_velocity_) {
        angular_velocity = (angular_velocity > 0 ? max_angular_velocity_ : -max_angular_velocity_);
    }
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.x = linear_velocity;
    cmd_vel->angular.z = angular_velocity;
 
    RCLCPP_INFO(this->logger_, "target pose in Odom x: %f, y: %f, yaw: %f", target_x_, target_y_, target_yaw_);
    RCLCPP_INFO(this->logger_, "current pose x: %f, y: %f", current_pose.pose.position.x, current_pose.pose.position.y);
    RCLCPP_INFO(this->logger_, "current pose z: %f, w: %f", current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    RCLCPP_INFO(this->logger_, "Distance to goal: %f, yaw error: %f", distance_error, orientation_error);
    RCLCPP_INFO(this->logger_, "pub vel x: %f, w: %f", cmd_vel->linear.x, cmd_vel->angular.z);
    this->vel_pub_->publish(std::move(cmd_vel));
    auto collision_monitor_switch = std::make_unique<std_msgs::msg::Bool>();
    collision_monitor_switch->data = false;
    collision_monito_switch_pub_->publish(std::move(collision_monitor_switch));
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

