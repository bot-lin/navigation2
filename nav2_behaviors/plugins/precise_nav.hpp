#ifndef NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_
#define NAV2_BEHAVIORS__PLUGINS__PRECISENAV_HPP_

#include <chrono>
#include <string>
#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/precise_nav.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class PIDController {
    double Kp, Ki, Kd, pid_i_max_;
    double integralError = 0;
    double prevError = 0;
public:
    PIDController(){}

    void setParams(double p, double i, double d, double i_max)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        pid_i_max_ = i_max;
    }


    void reset_pid()
    {
        integralError = 0;
        prevError = 0;
    }

    double compute(double setpoint, double actualValue) {
        double error = setpoint - actualValue;
        integralError += error;
        if (integralError > pid_i_max_) integralError = pid_i_max_;
        if (integralError < -pid_i_max_) integralError = -pid_i_max_;
        double derivativeError = error - prevError;
        prevError = error;
        return Kp * error + Ki * integralError + Kd * derivativeError;
    }
};

class SmoothController {
private:
    double previousOutput;
    double smoothingFactor;

public:
    SmoothController(double initialOutput = 0, double smoothingFactor = 0.1)
        : previousOutput(initialOutput), smoothingFactor(smoothingFactor) {}

    void setParams(double factor)
    {
        smoothingFactor = factor;
        previousOutput = 0.0;
    }
   
    double smooth(double currentOutput) {
        double smoothedOutput = (smoothingFactor * currentOutput) + 
                                ((1 - smoothingFactor) * previousOutput);
        previousOutput = smoothedOutput;
        return smoothedOutput;
    }
};

double find_v_based_on_w(double angleError, double k, double maxLinearVelocity, double scaleFactor, double distanceError, double d_max)
{      
    double distanceFactor = std::min(1.0, distanceError / d_max);
    // double v = maxLinearVelocity * (1 - k * std::abs(angleError)) * distanceFactor;
    
    // double v = maxLinearVelocity * (1 - k * std::abs(angleError));
    double v = maxLinearVelocity * (1 - k * std::tanh(scaleFactor * std::abs(angleError))) * distanceFactor;

    if (v < 0) v = 0;
    return v;
}
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
    double target_x_in_robot_frame_;
    double distance_goal_tolerance_ = 0.03;
    double heading_tolerance_ = 0.1;
    double yaw_goal_tolerance_ = 0.1;
    double angular_velocity_ = 0.2;
    double linear_velocity_ = 0.04;
    double orientation_p_ = 0.1;
    double max_linear_ = 0.5;
    double max_angular_ = 0.5;
    double distance_max_ = 1.0;
    double scale_factor_ = 2.0;
    bool reached_distance_goal_ = false;
    bool is_reverse_ = false;
    bool allow_reverse_ = true;
    bool is_heading_only_ = false;
    bool pid_reset_ = false;
    double steepness_ = 4.0;
    rclcpp::Duration command_time_allowance_{0, 0};
    PIDController angularController_;
    SmoothController smoothController_;
    std::string target_tf_frame_ = "odom";
    rclcpp::Time end_time_;
    
};
}
#endif