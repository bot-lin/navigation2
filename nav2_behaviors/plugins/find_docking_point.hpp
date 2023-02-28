#ifndef NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_
#define NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/precise_nav.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/PoseStamped.hpp"
#include "visualization_msgs/msg/marker.hpp"


namespace nav2_behaviors
{
using PreciseNavAction = nav2_msgs::action::PreciseNav;

class PreciseNav : public TimedBehavior<PreciseNavAction>
{
public:
    PreciseNav();
    ~PreciseNav();
    Status onRun(const std::shared_ptr<const PreciseNavAction::Goal> command) override;
    void onConfigure() override;
    Status onCycleUpdate() override;
    void find_docking_spot();

protected:
    PreciseNavAction::Feedback::SharedPtr feedback_;
    double target_x_;
    double target_y_;
    double target_yaw_;
    double distance_goal_tolerance_ = 0.03;
    double heading_tolerance_ = 0.1;
    double yaw_goal_tolerance_ = 0.1;
    double angular_velocity_ = 0.2;
    double linear_velocity_ = 0.04;
    bool reached_distance_goal_ = false;
    rclcpp::Client<zbot_interfaces::srv::LineSegmentListSrv>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    
};
}
#endif