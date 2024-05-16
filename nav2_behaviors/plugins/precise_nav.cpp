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

double find_v_based_on_w(double angleError, double k, double maxLinearVelocity, double distanceError, double d_max)
{      
    double distanceFactor = std::min(1.0, distanceError / d_max);
    double v = maxLinearVelocity * (1 - k * std::abs(angleError)) * distanceFactor;

    if (v < 0) v = 0;
    return v;
}


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
        "angular_velocity", rclcpp::ParameterValue(0.5));
    node->get_parameter("angular_velocity", angular_velocity_);

    nav2_util::declare_parameter_if_not_declared(
        node,
        "linear_velocity", rclcpp::ParameterValue(0.042));
    node->get_parameter("linear_velocity", linear_velocity_);

    nav2_util::declare_parameter_if_not_declared(
        node,
        "orientation_p", rclcpp::ParameterValue(0.1));
    node->get_parameter("orientation_p", orientation_p_);
    RCLCPP_INFO(this->logger_, "******************* %f", yaw_goal_tolerance_);


}

Status PreciseNav::onRun(const std::shared_ptr<const PreciseNavAction::Goal> command) 
{
    reached_distance_goal_ = false;
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
    is_heading_only_ = command->heading_only;
    target_tf_frame_ = command->target_tf_frame;
    

    is_reverse_ = command->is_reverse;
    allow_reverse_ = command->allow_reverse;
    
    yaw_goal_tolerance_ = command->yaw_goal_tolerance;
    distance_goal_tolerance_ = command->distance_goal_tolerance;
    angularController_.reset_pid();
    angularController_.setParams(command->orientation_p, command->orientation_i, command->orientation_d, 0.0);
    smoothController_.setParams(command->smoothing_factor);
    max_linear_ = command->max_linear;
    max_angular_ = command->max_angular;
    scale_factor_ = command->scale_factor;
    distance_max_ = command->distance_max;
    RCLCPP_INFO(this->logger_, "From precise nav");
    RCLCPP_INFO(this->logger_, "Is reverse %d", is_reverse_);
    RCLCPP_INFO(this->logger_, "Allow reverse %d", allow_reverse_);
    RCLCPP_INFO(this->logger_, "distance_goal_tolerance %f", distance_goal_tolerance_);
    RCLCPP_INFO(this->logger_, "yaw_goal_tolerance %f", yaw_goal_tolerance_);
    RCLCPP_INFO(this->logger_, "is heading only  %d", is_heading_only_);

    RCLCPP_INFO(this->logger_, "Received pose x: %f, y: %f", pose_tmp.pose.position.x, pose_tmp.pose.position.y);
    RCLCPP_INFO(this->logger_, "Received pose z: %f, w: %f", pose_tmp.pose.orientation.z, pose_tmp.pose.orientation.w);

    if (command->pose.header.frame_id != target_tf_frame_)
    {
        bool tf_response = nav2_util::transformPoseInTargetFrame(pose_tmp, pose_tmp,  *this->tf_, target_tf_frame_, this->transform_tolerance_);
        if (!tf_response)
        {
            RCLCPP_ERROR(this->logger_, "Failed to transform goal pose in %s frame from %s", target_tf_frame_.c_str(), command->pose.header.frame_id.c_str());
            return Status::FAILED;
        }
        RCLCPP_INFO(this->logger_, "Converting goal pose in %s frame from %s", target_tf_frame_.c_str(), command->pose.header.frame_id.c_str());
        RCLCPP_INFO(this->logger_, "Converted pose x: %f, y: %f", pose_tmp.pose.position.x, pose_tmp.pose.position.y);
    }
    RCLCPP_INFO(this->logger_, "target pose in %s x: %f, y: %f", target_tf_frame_.c_str(), pose_tmp.pose.position.x, pose_tmp.pose.position.y);

    // else{ 
    //     pose_tmp = command->pose;
    // }
    target_x_ = pose_tmp.pose.position.x;
    target_y_ = pose_tmp.pose.position.y;
    tf2::Quaternion q(pose_tmp.pose.orientation.x, 
                    pose_tmp.pose.orientation.y, 
                    pose_tmp.pose.orientation.z,
                    pose_tmp.pose.orientation.w);
    tf_response = nav2_util::transformPoseInTargetFrame(pose_tmp, pose_tmp,  *this->tf_, this->robot_base_frame_, this->transform_tolerance_);
    if (!tf_response)
    {
        RCLCPP_ERROR(this->logger_, "Failed to transform goal pose in %s frame from %s", this->robot_base_frame_.c_str(), command->pose.header.frame_id.c_str());
        return Status::FAILED;
    }
    target_x_in_robot_frame_ = pose_tmp.pose.position.x;
    RCLCPP_INFO(this->logger_, "target x in robot frame: %f", target_x_in_robot_frame_);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    target_yaw_ = yaw;
    if (command->reverse_yaw){
        target_yaw_ = -target_yaw_;
    }
    command_time_allowance_ = command->time_allowance;
    end_time_ = steady_clock_.now() + command_time_allowance_;
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
    is_heading_only_ = command->heading_only;
    target_tf_frame_ = command->target_tf_frame;
    
    is_reverse_ = command->is_reverse;
    allow_reverse_ = command->allow_reverse;
    angularController_.reset_pid();
    angularController_.setParams(command->orientation_p, command->orientation_i, command->orientation_d, 0.0);
    smoothController_.setParams(command->smoothing_factor);

    max_linear_ = command->max_linear;
    max_angular_ = command->max_angular;
    scale_factor_ = command->scale_factor;
    distance_max_ = command->distance_max;
    if (command->pose.header.frame_id != target_tf_frame_.c_str())
    {
        bool tf_response = nav2_util::transformPoseInTargetFrame(pose_tmp, pose_tmp,  *this->tf_, target_tf_frame_.c_str(), this->transform_tolerance_);
        if (!tf_response)
        {
            RCLCPP_ERROR(this->logger_, "Failed to transform goal pose in %s frame from %s", target_tf_frame_.c_str(), command->pose.header.frame_id.c_str());
            return Status::FAILED;
        }
        RCLCPP_INFO(this->logger_, "Converting goal pose in %s frame from %s", target_tf_frame_.c_str(), command->pose.header.frame_id.c_str());
        
    }
    else{
        pose_tmp = command->pose;
    }
    RCLCPP_INFO(this->logger_, "target pose in %s x: %f, y: %f", target_tf_frame_.c_str(), pose_tmp.pose.position.x, pose_tmp.pose.position.y);
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
    tf_response = nav2_util::transformPoseInTargetFrame(pose_tmp, pose_tmp,  *this->tf_, this->robot_base_frame_, this->transform_tolerance_);
    if (!tf_response)
    {
        RCLCPP_ERROR(this->logger_, "Failed to transform goal pose in %s frame from %s", this->robot_base_frame_.c_str(), command->pose.header.frame_id.c_str());
        return Status::FAILED;
    }
    target_x_in_robot_frame_ = pose_tmp.pose.position.x;
    RCLCPP_INFO(this->logger_, "target x in robot frame: %f", target_x_in_robot_frame_);
    reached_distance_goal_ = false;
    command_time_allowance_ = command->time_allowance;
    end_time_ = steady_clock_.now() + command_time_allowance_;
    return Status::SUCCEEDED;
}
Status PreciseNav::onCycleUpdate()
{
    rclcpp::Duration time_remaining = end_time_ - steady_clock_.now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
        stopRobot();
        RCLCPP_WARN(
        logger_,
        "Exceeded time allowance before reaching the Precise goal - Exiting Precise moving");
        return Status::FAILED;
    }
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *this->tf_, target_tf_frame_.c_str(), this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }
    double distance_to_goal = getDistanceToGoal(current_pose);
    double heading_error = getHeadingErrorToGoal(current_pose);
    double yaw_goal_error = getRadiansToGoal(current_pose);

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    if (is_heading_only_){
        RCLCPP_INFO(this->logger_, "reached_distance_goal_: %f", yaw_goal_error);

        if (std::fabs(yaw_goal_error) > yaw_goal_tolerance_){
            cmd_vel->linear.x = 0.0;
            cmd_vel->angular.z = std::clamp(angularController_.compute(0.0, -yaw_goal_error), -max_angular_, max_angular_);
            RCLCPP_INFO(this->logger_, "pub vel x: %f, w: %f", cmd_vel->linear.x, cmd_vel->angular.z);
            // if (heading_error > 0) cmd_vel->angular.z = angular_velocity_;
            // else cmd_vel->angular.z = -angular_velocity_;
        }
        else{
            this->stopRobot();
            return Status::SUCCEEDED;
        }
    }
    else{
        if (distance_to_goal > distance_goal_tolerance_ && !reached_distance_goal_)
        {

            cmd_vel->linear.x = find_v_based_on_w(heading_error, scale_factor_, max_linear_, steepness_, distance_to_goal, distance_max_);
            if (cmd_vel->linear.x > 0 && cmd_vel->linear.x < 0.015) cmd_vel->linear.x = 0.015;
            double pid_w = angularController_.compute(0.0, -heading_error);
            double smoothed_w = smoothController_.smooth(pid_w);
            cmd_vel->angular.z = std::clamp(smoothed_w, -max_angular_, max_angular_);
            if (allow_reverse_ && !is_reverse_ && target_x_in_robot_frame_ < 0.0) cmd_vel->linear.x = -cmd_vel->linear.x;
            if (is_reverse_) cmd_vel->linear.x = -cmd_vel->linear.x;
            // if (std::fabs(heading_error) > heading_tolerance_){
            //     if (is_reverse_) cmd_vel->linear.x = -0.01;
            //     else cmd_vel->linear.x = 0.01;
            //     if (heading_error > 0) cmd_vel->angular.z = angular_velocity_;
            //     else cmd_vel->angular.z = -angular_velocity_;
            // }
            // else
            // {
            //     if (is_reverse_) cmd_vel->linear.x = -linear_velocity_;
            //     else cmd_vel->linear.x = linear_velocity_;
            //     cmd_vel->angular.z = orientation_p_ * heading_error;
            // }
        }
        else if (std::fabs(yaw_goal_error) > yaw_goal_tolerance_)
        {
            cmd_vel->linear.x = 0.0;
            cmd_vel->angular.z = std::clamp(angularController_.compute(0.0, -yaw_goal_error), -max_angular_, max_angular_);
          
            
            reached_distance_goal_ = true;
        }
        else
        {
            reached_distance_goal_ = false;
            this->stopRobot();
            return Status::SUCCEEDED;
        }
        RCLCPP_INFO(this->logger_, "target pose in %s x: %f, y: %f, yaw: %f", target_tf_frame_.c_str(), target_x_, target_y_, target_yaw_);
        RCLCPP_INFO(this->logger_, "current pose x: %f, y: %f", current_pose.pose.position.x, current_pose.pose.position.y);
        RCLCPP_INFO(this->logger_, "current pose z: %f, w: %f", current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        RCLCPP_INFO(this->logger_, "Distance to goal: %f, yaw error: %f", distance_to_goal, yaw_goal_error);
        RCLCPP_INFO(this->logger_, "reached_distance_goal_: %d", reached_distance_goal_);
        RCLCPP_INFO(this->logger_, "pub vel x: %f, w: %f", cmd_vel->linear.x, cmd_vel->angular.z);
    }

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


    
    if ((is_reverse_) || (allow_reverse_ && !is_reverse_ && target_x_in_robot_frame_ < 0.0)){
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

