#include <cmath>
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "find_docking_point.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/robot_utils.hpp"


using namespace std::chrono_literals;

namespace nav2_behaviors
{

FindDockingPoint::FindDockingPoint()
: TimedBehavior<FindDockingPointAction>(),
    feedback_(std::make_shared<FindDockingPointAction::Feedback>()),
    distance_to_point_(0.35)
{

    

}

FindDockingPoint::~FindDockingPoint() = default;

void FindDockingPoint::onConfigure()
{
    my_node_ = node_.lock();
    if (!my_node_) {
        throw std::runtime_error{"Failed to lock node"};
    }
    nav2_util::declare_parameter_if_not_declared(
        my_node_,
        "distance_to_point", rclcpp::ParameterValue(0.3));
    my_node_->get_parameter("distance_to_point", distance_to_point_);

    client_ = my_node_->create_client<zbot_interfaces::srv::LineSegmentListSrv>("get_line_from_laser");
    publisher_ = my_node_->create_publisher<visualization_msgs::msg::Marker>("docking_point", 10);

}

Status FindDockingPoint::onRun(const std::shared_ptr<const FindDockingPointAction::Goal> command)
{
    distance_to_point_ = command->distance_to_point;

    return Status::SUCCEEDED;
}

Status FindDockingPoint::onCycleUpdate()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start finding docking point");
    // geometry_msgs::msg::PoseStamped current_pose;
    // if (!nav2_util::getCurrentPose(
    //     current_pose, *this->tf_, "map", this->robot_base_frame_,
    //     this->transform_tolerance_))
    // {
    //   RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
    //   return Status::FAILED;
    // }
    // double distance_to_goal = getDistanceToGoal(current_pose);
    // double heading_error = getHeadingErrorToGoal(current_pose);
    // double yaw_goal_error = getRadiansToGoal(current_pose);

    // auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    // if (distance_to_goal > distance_goal_tolerance_ && !reached_distance_goal_)
    // {
    //     if (std::fabs(heading_error) > heading_tolerance_){
    //         cmd_vel->linear.x = 0.005;
    //         if (heading_error > 0) cmd_vel->angular.z = angular_velocity_;
    //         else cmd_vel->angular.z = -angular_velocity_;
    //     }
    //     else
    //     {
    //         cmd_vel->linear.x = linear_velocity_;
    //     }
    // }
    // else if (std::fabs(yaw_goal_error) > yaw_goal_tolerance_)
    // {
    //     cmd_vel->angular.z = 0.3 * yaw_goal_error;
    //     reached_distance_goal_ = true;
    // }
    // else
    // {
    //     reached_distance_goal_ = false;
    //     this->stopRobot();
    //     return Status::SUCCEEDED;
    // }

    // this->vel_pub_->publish(std::move(cmd_vel));
    find_docking_spot();
    return Status::RUNNING;
}

void FindDockingPoint::find_docking_spot()
{
    auto request = std::make_shared<zbot_interfaces::srv::LineSegmentListSrv::Request>();
    request->request = true;
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Start calling find line request");

    auto result = client_->async_send_request(request);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "1");

    if (rclcpp::spin_until_future_complete(my_node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "2");
        std::vector<zbot_interfaces::msg::LineSegment> lines = result.get()->line_segments;
        auto start = lines[0].start;
        auto end = lines[1].end;
        double x2 = start[0];
        double y2 = start[1];
        double x1 = end[0];
        double y1 = end[1];
        double tmp = distance_to_point_ /std::sqrt(std::pow(y1-y2, 2) + std::pow(x1-x2, 2));
        double x3 = (x1 + x2) /2 - tmp * (y1 -y2);
        double y3 = (y1 + y2) /2 - tmp * (x2 - x1);
        geometry_msgs::msg::PoseStamped pose_laser, pose_map;
        pose_laser.header.frame_id = "laser";
        pose_laser.pose.position.x = x3;
        pose_laser.pose.position.y = y3;
        pose_laser.pose.orientation.z = 1.0;
        nav2_util::transformPoseInTargetFrame(pose_laser, pose_map,  *this->tf_, "map");


        visualization_msgs::msg::Marker markers_msg;
        markers_msg.header.frame_id = "laser";
        markers_msg.type = 0;
        markers_msg.id = 0;
        markers_msg.scale.x = 0.03;
        markers_msg.scale.y = 0.03;
        markers_msg.scale.z = 0.03;
        markers_msg.color.r = 0.0;
        markers_msg.color.g = 1.0;
        markers_msg.color.b = 0.0;
        markers_msg.color.a = 1.0;
        geometry_msgs::msg::Point point_msg;
        point_msg.x = x3;
        point_msg.y = y3;
        point_msg.z = 0.0;
        markers_msg.points.push_back(point_msg);
        publisher_->publish(markers_msg);

    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get line from laser");
    }
}





} // namespace name
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::FindDockingPoint, nav2_core::Behavior)

