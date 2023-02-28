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
#include "zbot_interfaces/srv/line_segment_list_srv.hpp"
#include "zbot_interfaces/msg/line_segment.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_util/robot_utils.hpp"


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

    publisher_ = node.create_publisher<visualization_msgs::msg::Marker>("docking_point", 10);

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

    rclcpp::Client<zbot_interfaces::srv::LineSegmentListSrv>::SharedPtr client_ =
    node->create_client<zbot_interfaces::srv::LineSegmentListSrv>("get_line_from_laser");

}

Status PreciseNav::onRun(const std::shared_ptr<const PreciseNavAction::Goal> command)
{
    target_x_ = command->pose.pose.position.x;
    target_y_ = command->pose.pose.position.y;
    tf2::Quaternion q(command->pose.pose.orientation.x, 
                    command->pose.pose.orientation.y, 
                    command->pose.pose.orientation.z,
                    command->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    target_yaw_ = yaw;
    return Status::SUCCEEDED;
}

Status PreciseNav::onCycleUpdate()
{
    geometry_msgs::msg::PoseStamped current_pose;
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

void PreciseNav::find_docking_spot()
{
    auto request = std::make_shared<zbot_interfaces::srv::LineSegmentListSrv::Request>();
    request->request = true;
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        std::vector<LineSegment> lines = result.get()->line_segments;
        std::vector<double> start = lines[0].start;
        std::vector<double> end = lines[1].end;
        double x2 = start[0];
        double y2 = start[1];
        double x1 = end[0];
        double y1 = end[1];
        double tmp = distance_ /std::sqrt(std::pow(y1-y2) + std::pow(x1-x2));
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
        markers_msg.scale.x = 0.03
        markers_msg.scale.y = 0.03
        markers_msg.scale.z = 0.03
        markers_msg.color.r = 0.0
        markers_msg.color.g = 1.0
        markers_msg.color.b = 0.0
        markers_msg.color.a = 1.0
        geometry_msgs::msg::Point poit_msg;
        poit_msg.x = x3;
        poit_msg.y = y3;
        poit_msg.z = 0.0;
        markers_msg.points.push_back(point_msg);
        pub_->publish(markers_msg);

    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get line from laser");
    }
}





} // namespace name
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::PreciseNav, nav2_core::Behavior)

