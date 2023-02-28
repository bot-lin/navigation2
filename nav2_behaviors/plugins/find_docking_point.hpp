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
using FindDockingPointAction = nav2_msgs::action::FindDockingPoint;

class FindDockingPoint : public TimedBehavior<FindDockingPointAction>
{
public:
    FindDockingPoint();
    ~FindDockingPoint();
    Status onRun(const std::shared_ptr<const FindDockingPointAction::Goal> command) override;
    void onConfigure() override;
    Status onCycleUpdate() override;
    void find_docking_spot();

protected:
    FindDockingPointAction::Feedback::SharedPtr feedback_;
    double distance_to_point_;
    rclcpp::Client<zbot_interfaces::srv::LineSegmentListSrv>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    
};
}
#endif