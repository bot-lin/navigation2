// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace nav2_regulated_pure_pursuit_controller
{

void RegulatedPurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  double transform_tolerance = 0.1;
  goal_dist_tol_ = 0.25;  // reasonable default before first update
  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".curvature_lookahead_dist", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".approach_velocity_scaling_dist",
    rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_collision_detection",
    rclcpp::ParameterValue(true));
 
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
  
  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
  
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".move_reversing", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".allow_y", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist",
    rclcpp::ParameterValue(getCostmapMaxExtent()));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".use_interpolation",
    rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid_p", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid_i", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid_d", rclcpp::ParameterValue(0.001));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid_i_max", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid_scaling_factor", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".pid_steepness_control", rclcpp::ParameterValue(2.0));

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(10.0));
  node->get_parameter(plugin_name_ + ".regulated_linear_scaling_min_radius", regulated_linear_scaling_min_radius_);
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  base_desired_linear_vel_ = desired_linear_vel_;
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
  node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
  node->get_parameter(
    plugin_name_ + ".max_angular_vel",
    max_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".curvature_lookahead_dist", curvature_lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".regulated_linear_scaling_min_speed",
    regulated_linear_scaling_min_speed_);
  node->get_parameter(
    plugin_name_ + ".use_velocity_scaled_lookahead_dist",
    use_velocity_scaled_lookahead_dist_);
  node->get_parameter(
    plugin_name_ + ".min_approach_linear_velocity",
    min_approach_linear_velocity_);
  node->get_parameter(
    plugin_name_ + ".approach_velocity_scaling_dist",
    approach_velocity_scaling_dist_);
  if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
    RCLCPP_WARN(
      logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
      "leading to permanent slowdown");
  }
  node->get_parameter(
    plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
    max_allowed_time_to_collision_up_to_carrot_);
  node->get_parameter(
    plugin_name_ + ".use_collision_detection",
    use_collision_detection_);
 
  node->get_parameter(
    plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
    use_cost_regulated_linear_velocity_scaling_);
  node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
  node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
  node->get_parameter(
    plugin_name_ + ".inflation_cost_scaling_factor",
    inflation_cost_scaling_factor_);

  
  node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
  node->get_parameter(plugin_name_ + ".move_reversing", move_reversing_);
  node->get_parameter(plugin_name_ + ".allow_y", allow_y_);
  node->get_parameter(
    plugin_name_ + ".max_robot_pose_search_dist",
    max_robot_pose_search_dist_);
  node->get_parameter(
    plugin_name_ + ".use_interpolation",
    use_interpolation_);
  
  node->get_parameter(plugin_name_ + ".pid_p", pid_p_);
  node->get_parameter(plugin_name_ + ".pid_i", pid_i_);
  node->get_parameter(plugin_name_ + ".pid_d", pid_d_);
  node->get_parameter(plugin_name_ + ".pid_i_max", pid_i_max_);
  node->get_parameter(plugin_name_ + ".pid_scaling_factor", pid_scaling_factor_);
  node->get_parameter(plugin_name_ + ".pid_steepness_control", pid_steepness_control_);
  angularController_.setParams(pid_p_, pid_i_, pid_d_, pid_i_max_);
  smoothController_.setParams(0.7);

  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  if (inflation_cost_scaling_factor_ <= 0.0) {
    RCLCPP_WARN(
      logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
      "it should be >0. Disabling cost regulated linear velocity scaling.");
    use_cost_regulated_linear_velocity_scaling_ = false;
  }


  global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
  carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
  turning_radius_pub_ = node->create_publisher<std_msgs::msg::Float32>("rpp_turning_radius", 1);
  collision_pub_ = node->create_publisher<std_msgs::msg::Bool>("rpp/collision", 1);
  change_param_sub_ = node->create_subscription<zbot_interfaces::msg::ChangeParam>(
    "rpp_pid/change_param", rclcpp::QoS(10),
    std::bind(&RegulatedPurePursuitController::changeParamCallback, this, std::placeholders::_1));

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  collision_checker_->setCostmap(costmap_);
}

void RegulatedPurePursuitController::changeParamCallback(const zbot_interfaces::msg::ChangeParam::SharedPtr msg)
{
  int param_type = msg->param_type;
  RCLCPP_INFO(
    logger_,
    "change Param type: %d name: %s value: %f", param_type, msg->param_name.c_str(), msg->param_double_value);
  if (msg->param_name == "pid_p")
  {
    pid_p_ = msg->param_double_value;
    angularController_.setParams(pid_p_, pid_i_, pid_d_, pid_i_max_);
  } else if (msg->param_name == "pid_i")
  {
    pid_i_ = msg->param_double_value;
    angularController_.setParams(pid_p_, pid_i_, pid_d_, pid_i_max_);
  } else if (msg->param_name == "pid_d")
  {
    pid_d_ = msg->param_double_value;
    angularController_.setParams(pid_p_, pid_i_, pid_d_, pid_i_max_);
  } else if (msg->param_name == "pid_i_max")
  {
    pid_i_max_ = msg->param_double_value;
    angularController_.setParams(pid_p_, pid_i_, pid_d_, pid_i_max_);
  } else if (msg->param_name == "pid_scaling_factor")
  {
    pid_scaling_factor_ = msg->param_double_value;
  } else if (msg->param_name == "pid_steepness_control")
  {
    pid_steepness_control_ = msg->param_double_value;
  } else if (msg->param_name == "curvature_lookahead_dist")
  {
    curvature_lookahead_dist_ = msg->param_double_value;
  } else if (msg->param_name == "max_lookahead_dist")
  {
    max_lookahead_dist_ = msg->param_double_value;
  } else if (msg->param_name == "min_lookahead_dist")
  {
    min_lookahead_dist_ = msg->param_double_value;
  } else if (msg->param_name == "lookahead_time")
  {
    lookahead_time_ = msg->param_double_value;
  } else if (msg->param_name == "regulated_linear_scaling_min_radius")
  {
    regulated_linear_scaling_min_radius_ = msg->param_double_value;
  } else if (msg->param_name == "max_angular_vel")
  {
    max_angular_vel_ = msg->param_double_value;
  } else if (msg->param_name == "allow_y")
  {
    allow_y_ = msg->param_bool_value;
  }
  
  
}

void RegulatedPurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type"
    " regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_.reset();
  carrot_pub_.reset();
  carrot_arc_pub_.reset();
  turning_radius_pub_.reset();
  collision_pub_.reset();
}

void RegulatedPurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_activate();
  carrot_pub_->on_activate();
  carrot_arc_pub_->on_activate();
  turning_radius_pub_->on_activate();
  collision_pub_->on_activate();
  // Add callback for dynamic parameters
  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &RegulatedPurePursuitController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void RegulatedPurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());
  global_path_pub_->on_deactivate();
  carrot_pub_->on_deactivate();
  carrot_arc_pub_->on_deactivate();
  turning_radius_pub_->on_deactivate();
  dyn_params_handler_.reset();
  collision_pub_->on_deactivate();
}

std::unique_ptr<geometry_msgs::msg::PointStamped> RegulatedPurePursuitController::createCarrotMsg(
  const geometry_msgs::msg::PoseStamped & carrot_pose)
{
  auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  carrot_msg->header = carrot_pose.header;
  carrot_msg->point.x = carrot_pose.pose.position.x;
  carrot_msg->point.y = carrot_pose.pose.position.y;
  carrot_msg->point.z = 0.01;  // publish right over map to stand out
  return carrot_msg;
}

double RegulatedPurePursuitController::getLookAheadDistance(
  const geometry_msgs::msg::Twist & speed)
{
  // If using velocity-scaled look ahead distances, find and clamp the dist
  // Else, use the static look ahead distance
  double lookahead_dist = lookahead_dist_;
  if (use_velocity_scaled_lookahead_dist_) {
    lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
    lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);     
  }

  return lookahead_dist;
}
geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeRotateCommands(
  const double & yaw_diff)
{
  double angular_vel;
  double pid_w = angularController_.compute(0.0, -yaw_diff);
  double smooth_w = smoothController_.smooth(pid_w);
 
  angular_vel = std::clamp(smooth_w, -max_angular_vel_, max_angular_vel_);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

geometry_msgs::msg::TwistStamped RegulatedPurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{

  std::lock_guard<std::mutex> lock_reinit(mutex_);

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }




  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }


  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);

  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel, linear_y;

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  auto curvature_pose = getLookAheadPoint(curvature_lookahead_dist_, transformed_plan);
  const double curvature_dist2 =
  (curvature_pose.pose.position.x * curvature_pose.pose.position.x) +
  (curvature_pose.pose.position.y * curvature_pose.pose.position.y);
  if (curvature_dist2 > 0.001) {
  curvature = 2.0 * curvature_pose.pose.position.y / curvature_dist2;
  }
  
  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_ && !move_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  if (move_reversing_) {
    sign = -1.0;
  }

  //   if (allow_reversing_) {
  //   sign = -1.0;
  // }

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  getRadToCarrotPose(carrot_pose, angle_to_heading);

  
    
  
  if (!allow_y_){
    applyConstraints(
      curvature, angle_to_heading, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);
    double pid_w = angularController_.compute(0.0, -angle_to_heading);
    double smooth_w = smoothController_.smooth(pid_w);
    angular_vel = std::clamp(smooth_w, -max_angular_vel_, max_angular_vel_);
    linear_y = 0.0;
  }else{
    linear_y = linear_vel * sin(angle_to_heading);
    linear_vel = linear_vel * cos(angle_to_heading);
    angular_vel = 0.0;

  }

  std_msgs::msg::Bool msg;
  // Collision checking on this velocity heading
  // bool first_collision = true;
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
    msg.data = true;
    collision_pub_->publish(msg);
    // throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");

    RCLCPP_INFO(logger_, "RegulatedPurePursuitController detected collision ahead!");
    geometry_msgs::msg::PoseStamped transformed_pose;
    transformPose("base_link", collision_pose_msg_, transformed_pose);
    double x = transformed_pose.pose.position.x;
    double y = transformed_pose.pose.position.y;
    std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();
    double half_width = footprint[0].y;
    if (x > 0){//front
    double l_v = 0.05;
      if (abs(y)>half_width+0.01){
        linear_vel = 0.15;
        angular_vel = 0.0;
      }
      else if (y > 0)
      {
        linear_vel = l_v;
        angular_vel = -0.1;
      }
      else if (y < 0){
        linear_vel = l_v;
        angular_vel = 0.1;
      }
      else
      {
        linear_vel = -0.05;
        angular_vel = 0.0;
      }
      
    }
  }
  // RCLCPP_INFO(logger_, "linear %f", linear_vel);
  // RCLCPP_INFO(logger_, "angular %f", angular_vel);
  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.linear.y = linear_y;
  // if (!goal_checker->getCheckXY())
  // {
  //   cmd_vel.twist.linear.x = 0.0;
  // }
  
  cmd_vel.twist.angular.z = angular_vel;
  msg.data = false;
  collision_pub_->publish(msg);
  return cmd_vel;
}


void RegulatedPurePursuitController::getRadToCarrotPose(
  const geometry_msgs::msg::PoseStamped & carrot_pose, double & angle_to_path)
{
  // Whether we should rotate robot to rough path heading
  if (move_reversing_){
    angle_to_path = atan2(-carrot_pose.pose.position.y, -carrot_pose.pose.position.x);
  }
  else{
    angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  }

    if (allow_reversing_) {
    angle_to_path = carrot_pose.pose.position.x<0 ? atan2(-carrot_pose.pose.position.y, -carrot_pose.pose.position.x) : atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  }

}


geometry_msgs::msg::Point RegulatedPurePursuitController::circleSegmentIntersection(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2,
  double r)
{
  // Formula for intersection of a line with a circle centered at the origin,
  // modified to always return the point that is on the segment between the two points.
  // https://mathworld.wolfram.com/Circle-LineIntersection.html
  // This works because the poses are transformed into the robot frame.
  // This can be derived from solving the system of equations of a line and a circle
  // which results in something that is just a reformulation of the quadratic formula.
  // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
  // https://www.desmos.com/calculator/td5cwbuocd
  double x1 = p1.x;
  double x2 = p2.x;
  double y1 = p1.y;
  double y2 = p2.y;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // Augmentation to only return point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  geometry_msgs::msg::Point p;
  double sqrt_term = std::sqrt(r * r * dr2 - D * D);
  p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
  p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
  return p;
}

geometry_msgs::msg::PoseStamped RegulatedPurePursuitController::getLookAheadPoint(
  const double & lookahead_dist,
  const nav_msgs::msg::Path & transformed_plan)
{
  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto & ps) {
      return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

  // If the no pose is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.poses.end()) {
    goal_pose_it = std::prev(transformed_plan.poses.end());
  } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
    // Find the point on the line segment between the two poses
    // that is exactly the lookahead distance away from the robot pose (the origin)
    // This can be found with a closed form for the intersection of a segment and a circle
    // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
    // and goal_pose is guaranteed to be outside the circle.
    auto prev_pose_it = std::prev(goal_pose_it);
    auto point = circleSegmentIntersection(
      prev_pose_it->pose.position,
      goal_pose_it->pose.position, lookahead_dist);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = prev_pose_it->header.frame_id;
    pose.header.stamp = goal_pose_it->header.stamp;
    pose.pose.position = point;
    return pose;
  }

  return *goal_pose_it;
}

bool RegulatedPurePursuitController::isCollisionImminent(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const double & linear_vel, const double & angular_vel,
  const double & carrot_dist)
{
  // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
  // odom frame and the carrot_pose is in robot base frame.

  // check current point is OK
  if (inCollision(
      robot_pose.pose.position.x, robot_pose.pose.position.y,
      tf2::getYaw(robot_pose.pose.orientation)))
  {
    return true;
  }

  // visualization messages
  nav_msgs::msg::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;

  double projection_time = 0.0;
  if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01) {
    // rotating to heading at goal or toward path
    // Equation finds the angular distance required for the largest
    // part of the robot radius to move to another costmap cell:
    // theta_min = 2.0 * sin ((res/2) / r_max)
    // via isosceles triangle r_max-r_max-resolution,
    // dividing by angular_velocity gives us a timestep.
    double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
    projection_time =
      2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
  } else {
    // Normal path tracking
    projection_time = costmap_->getResolution() / fabs(linear_vel);
  }

  const geometry_msgs::msg::Point & robot_xy = robot_pose.pose.position;
  geometry_msgs::msg::Pose2D curr_pose;
  curr_pose.x = robot_pose.pose.position.x;
  curr_pose.y = robot_pose.pose.position.y;
  curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

  // only forward simulate within time requested
  int i = 1;
  while (i * projection_time < max_allowed_time_to_collision_up_to_carrot_) {
    i++;

    // apply velocity at curr_pose over distance
    curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
    curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
    curr_pose.theta += projection_time * angular_vel;

    // check if past carrot pose, where no longer a thoughtfully valid command
    if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist) {
      break;
    }

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
      geometry_msgs::msg::Pose2D collision_pose = collision_checker_->findCollsionPoint(
        curr_pose.x, curr_pose.y, curr_pose.theta, costmap_ros_->getRobotFootprint());
      collision_pose_msg_ = pose_msg;
      collision_pose_msg_.pose.position.x = collision_pose.x;
      collision_pose_msg_.pose.position.y = collision_pose.y;
      carrot_arc_pub_->publish(arc_pts_msg);
      return true;
    }
  }

  carrot_arc_pub_->publish(arc_pts_msg);

  return false;
}

bool RegulatedPurePursuitController::inCollision(
  const double & x,
  const double & y,
  const double & theta)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 30000,
      "The dimensions of the costmap is too small to successfully check for "
      "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
      "increase your costmap size.");
    return false;
  }

  double footprint_cost = collision_checker_->footprintCostAtPose(
    x, y, theta, costmap_ros_->getRobotFootprint());
  if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
    costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
  {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return footprint_cost >= static_cast<double>(LETHAL_OBSTACLE);
}

double RegulatedPurePursuitController::costAtPose(const double & x, const double & y)
{
  unsigned int mx, my;

  if (!costmap_->worldToMap(x, y, mx, my)) {
    RCLCPP_FATAL(
      logger_,
      "The dimensions of the costmap is too small to fully include your robot's footprint, "
      "thusly the robot cannot proceed further");
    throw nav2_core::PlannerException(
            "RegulatedPurePursuitController: Dimensions of the costmap are too small "
            "to encapsulate the robot footprint at current speeds!");
  }

  unsigned char cost = costmap_->getCost(mx, my);
  return static_cast<double>(cost);
}

double RegulatedPurePursuitController::approachVelocityScalingFactor(
  const nav_msgs::msg::Path & transformed_path
) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < approach_velocity_scaling_dist_) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / approach_velocity_scaling_dist_;
  } else {
    return 1.0;
  }
}

void RegulatedPurePursuitController::applyApproachVelocityScaling(
  const nav_msgs::msg::Path & path,
  double & linear_vel
) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < min_approach_linear_velocity_) {
    approach_vel = min_approach_linear_velocity_;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

void RegulatedPurePursuitController::applyConstraints(
  const double & curvature, const double angle_to_heading, const geometry_msgs::msg::Twist & /*curr_speed*/,
  const double & pose_cost, const nav_msgs::msg::Path & path, double & linear_vel, double & sign)
{
  double cost_vel = linear_vel;
  double curvature_vel = linear_vel;
  const double radius = fabs(1.0 / curvature);
  const double & min_radius = regulated_linear_scaling_min_radius_;
  if (radius < min_radius) {
    curvature_vel *= 1.0 - (fabs(radius - min_radius) / min_radius);
    curvature_vel = std::max(curvature_vel, regulated_linear_scaling_min_speed_);

  } 

  // limit the linear velocity by proximity to obstacles
  if (use_cost_regulated_linear_velocity_scaling_ &&
    pose_cost != static_cast<double>(NO_INFORMATION) &&
    pose_cost != static_cast<double>(FREE_SPACE))
  {
    const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
    const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
      std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) + inscribed_radius;

    if (min_distance_to_obstacle < cost_scaling_dist_) {
      cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
    }
  }
  
  // Limit linear velocities to be valid
  linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);

  double pid_regulated_vel = 0.0;
  pid_regulated_vel = find_v_based_on_w(angle_to_heading, pid_scaling_factor_, desired_linear_vel_, pid_steepness_control_);
  // linear_vel = std::min(pid_regulated_vel, linear_vel);
  linear_vel = std::min(cost_vel, pid_regulated_vel);
  linear_vel = std::min(curvature_vel, linear_vel);
  applyApproachVelocityScaling(path, linear_vel);
  linear_vel = sign * linear_vel;
}

void RegulatedPurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void RegulatedPurePursuitController::setSpeedLimit(
  const double & speed_limit,
  const bool & percentage)
{
  if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) { //0.0
    // Restore default value
    desired_linear_vel_ = desired_linear_vel_;
  } else if (speed_limit < nav2_costmap_2d::NO_SPEED_LIMIT)
  {
    /* code */
    desired_linear_vel_ = 0.0;
  } else {
    if (percentage) {
      // Speed limit is expressed in % from maximum speed of robot
      desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
    } else {
      // Speed limit is expressed in absolute value
      desired_linear_vel_ = speed_limit;
    }
  }
}

nav_msgs::msg::Path RegulatedPurePursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  double max_costmap_extent = getCostmapMaxExtent();

  auto closest_pose_upper_bound =
    nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  // First find the closest pose on the path to the robot
  // bounded by when the path turns around (if it does) so we don't get a pose from a later
  // portion of the path
  auto transformation_begin =
    nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // Find points up to max_transform_dist so we only transform them.
  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & pose) {
      return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

  // Lambda to transform a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = robot_pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
      transformed_pose.pose.position.z = 0.0;
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_path_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

double RegulatedPurePursuitController::findVelocitySignChange(
  const nav_msgs::msg::Path & transformed_plan)
{
  // Iterating through the transformed global path to determine the position of the cusp
  for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id) {
    // We have two vectors for the dot product OA and AB. Determining the vectors.
    double oa_x = transformed_plan.poses[pose_id].pose.position.x -
      transformed_plan.poses[pose_id - 1].pose.position.x;
    double oa_y = transformed_plan.poses[pose_id].pose.position.y -
      transformed_plan.poses[pose_id - 1].pose.position.y;
    double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
      transformed_plan.poses[pose_id].pose.position.x;
    double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
      transformed_plan.poses[pose_id].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
    if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) {
      // returning the distance if there is a cusp
      // The transformed path is in the robots frame, so robot is at the origin
      return hypot(
        transformed_plan.poses[pose_id].pose.position.x,
        transformed_plan.poses[pose_id].pose.position.y);
    }
  }

  return std::numeric_limits<double>::max();
}

bool RegulatedPurePursuitController::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double RegulatedPurePursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters = std::max(
    costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}


rcl_interfaces::msg::SetParametersResult
RegulatedPurePursuitController::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  RCLCPP_INFO(
    logger_,
    "Reconfigure Request for controller: %s of type "
    "regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    plugin_name_.c_str());

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".inflation_cost_scaling_factor") {
        if (parameter.as_double() <= 0.0) {
          RCLCPP_WARN(
            logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
            "it should be >0. Ignoring parameter update.");
          continue;
        }
        inflation_cost_scaling_factor_ = parameter.as_double();
      } else if (name == plugin_name_ + ".desired_linear_vel") {
        desired_linear_vel_ = parameter.as_double();
        base_desired_linear_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_dist") {
        lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_lookahead_dist") {
        max_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_lookahead_dist") {
        min_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".lookahead_time") {
        lookahead_time_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_angular_vel") {
        max_angular_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
        min_approach_linear_velocity_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot") {
        max_allowed_time_to_collision_up_to_carrot_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_dist") {
        cost_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cost_scaling_gain") {
        cost_scaling_gain_ = parameter.as_double();
      } else if (name == plugin_name_ + ".transform_tolerance") {
        double transform_tolerance = parameter.as_double();
        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
      } else if (name == plugin_name_ + ".pid_p") {
        pid_p_ = parameter.as_double();
      } else if (name == plugin_name_ + ".pid_i") {
        pid_i_ = parameter.as_double();
      } else if (name == plugin_name_ + ".pid_d") {
        pid_d_ = parameter.as_double();
      } else if (name == plugin_name_ + ".pid_i_max") {
        pid_i_max_ = parameter.as_double();
      } else if (name == plugin_name_ + ".pid_scaling_factor") {
        pid_scaling_factor_ = parameter.as_double();
      } else if (name == plugin_name_ + ".pid_steepness_control") {
        pid_steepness_control_ = parameter.as_double();
      } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
        approach_velocity_scaling_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".curvature_lookahead_dist") {
        curvature_lookahead_dist_ = parameter.as_double();
      } else if (name == plugin_name_ + ".regulated_linear_scaling_min_speed") {
        regulated_linear_scaling_min_speed_ = parameter.as_double();
      } else if (name == plugin_name_ + ".regulated_linear_scaling_min_radius") {
        regulated_linear_scaling_min_radius_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
        use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling") {
        use_cost_regulated_linear_velocity_scaling_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".move_reversing") {
        if (allow_reversing_ && parameter.as_bool()) {
          RCLCPP_WARN(
            logger_, "Both use_rotate_to_heading and allow_reversing "
            "parameter cannot be set to true. Rejecting parameter update.");
          continue;
        }
        move_reversing_ = parameter.as_bool();
      }
    }
  }
  
  result.successful = true;
  return result;
}

}  // namespace nav2_regulated_pure_pursuit_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController,
  nav2_core::Controller)
