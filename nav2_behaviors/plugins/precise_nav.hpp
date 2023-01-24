#ifndef NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_
#define NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/precise_nav.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

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

protected:
    PreciseNavAction::Feedback::SharedPtr feedback_;
    double xv_vel_;
    double zw_vel_;
    
};
}
#endif