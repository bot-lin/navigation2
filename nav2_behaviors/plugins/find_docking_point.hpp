#ifndef NAV2_BEHAVIORS__PLUGINS__FINDDOCKINGPOINT_HPP_
#define NAV2_BEHAVIORS__PLUGINS__FINDDOCKINGPOINT_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/find_docking_point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "zbot_interfaces/srv/line_segment_list_srv.hpp"
#include "zbot_interfaces/msg/line_segment.hpp"

namespace nav2_behaviors
{
struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
};

struct Quaternion {
    double w, x, y, z;
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
};
using FindDockingPointAction = nav2_msgs::action::FindDockingPoint;

class FindDockingPoint : public TimedBehavior<FindDockingPointAction>
{
public:
    FindDockingPoint();
    ~FindDockingPoint();
    Status onRun(const std::shared_ptr<const FindDockingPointAction::Goal> command) override;
    Status change_goal(const std::shared_ptr<const FindDockingPointAction::Goal> command) override;
    void onConfigure() override;
    Status onCycleUpdate() override;
    bool find_docking_spot();
    Point findClockwisePerpendicularVector(const Point& A, const Point& B);
    Quaternion vectorToQuaternion(const Point& vec);

   

protected:
    FindDockingPointAction::Feedback::SharedPtr feedback_;
    FindDockingPointAction::Result::SharedPtr result_;
    double distance_to_point_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> my_node_;
    rclcpp::Client<zbot_interfaces::srv::LineSegmentListSrv>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    bool processing_;
    
};
}
#endif