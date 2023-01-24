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
    target_x_(0.0),
    target_y_(0.0),
    target_yaw_(0.0)
{
}

PreciseNav::~PreciseNav() = default;

void PreciseNav::onConfigure()
{

}

Status PreciseNav::onRun(const std::shared_ptr<const PreciseNavAction::Goal> command)
{
    target_x_ = command->target.x;
    target_y_ = command->target.y;
    target_yaw_ = command->target.z;

    if (!nav2_util::getCurrentPose(
        initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }
    return Status::SUCCEEDED;
}

Status PreciseNav::onCycleUpdate()
{
    return Status::RUNNING;
}


} // namespace name
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_behaviors::PreciseNav, nav2_core::Behavior)

