#ifndef NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_
#define NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/precise_nav.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class PIDController {
    double Kp, Ki, Kd;
    double integralError = 0;
    double prevError = 0;

public:
    PIDController(double p, double i, double d) : Kp(p), Ki(i), Kd(d) {}

    void setParams(double p, double i, double d)
    {
        Kp = p;
        Ki = i;
        Kd = d;
    }

    void reset_pid()
    {
        integralError = 0;
        prevError = 0;
    }

    double compute(double setpoint, double actualValue) {
        double error = setpoint - actualValue;
        integralError += error;
        double derivativeError = error - prevError;
        prevError = error;
        return Kp * error + Ki * integralError + Kd * derivativeError;
    }
};

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
    Status change_goal(const std::shared_ptr<const PreciseNavAction::Goal> command) override;
    double getDistanceToGoal(geometry_msgs::msg::PoseStamped current_pose);
    double getHeadingErrorToGoal(geometry_msgs::msg::PoseStamped current_pose);
    double getRadiansToGoal(geometry_msgs::msg::PoseStamped current_pose);

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
    double orientation_p_ = 0.1;
    bool reached_distance_goal_ = false;
    bool is_reverse_ = false;
    bool is_heading_only_ = false;
    PIDController angularController_(1.0, 0.0, 0.5);
    std::string target_tf_frame_ = "odom";
    
};
}
#endif