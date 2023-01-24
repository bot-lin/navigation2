#include <cmath>
#include <thread>
#include <algorithm>
#include <memory>
#include <utility>

#include "precise_nav.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace nav2_behaviors
{

PreciseNav::PreciseNav()
: TimedBehavior<PreciseNavAction>(),
    feedback_(std::make_shared<PreciseNavAction::Feedback>()),
    xv_vel_(0.0),
    zw_vel_(0.0)
{
}

PreciseNav::~PreciseNav() = default;

void PreciseNav::onConfigure()
{

}

Status PreciseNav::onRun(const std::shared_ptr<const PreciseNavAction::Goal> command)
{
    float target_x = command -> target_x;
    return Status::SUCCEEDED;
}

Status PreciseNav::onCycleUpdate()
{
    return Status::RUNNING;
}


} // namespace name
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::PreciseNav, nav2_core::Behavior)

