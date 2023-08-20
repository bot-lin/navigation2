#ifndef NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_
#define NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/precise_nav.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class PIDController {
private:
    double Kp, Ki, Kd;
    double previous_error, integral;

public:
    PIDController() 
        : previous_error(0), integral(0) {}
    
    void initPID(double p, double i, double d) {
        Kp = p;
        Ki = i;
        Kd = d;
        previous_error = 0.0;
        integral = 0.0;
    }
 
    double compute(double setpoint, double actual_value, double dt) {
        double error = setpoint - actual_value;
        integral += error * dt;
        double derivative = (error - previous_error) / dt;
        previous_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
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
    double yaw_goal_tolerance_ = 0.1;
    double max_angular_velocity_ = 0.2;
    double max_linear_velocity_ = 0.05;
    
    PIDController position_controller_, orientation_controller_;

    bool is_reverse_ = false;
    rclcpp::Time last_pid_time_;
    
};
}
#endif