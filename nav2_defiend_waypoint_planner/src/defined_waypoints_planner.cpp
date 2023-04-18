/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "nav2_util/node_utils.hpp"

#include "nav2_definedwaypoints_planner/defined_waypoints_planner.hpp"
using namespace cv;

namespace nav2_definedwaypoints_planner
{

void DefinedWaypoints::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  origin_x_ = costmap_ros->getOriginX();
  origin_y_ = costmap_ros->getOriginY();
  resolution_  = costmap_ros->getResolution();
  size_x_ = costmap_ros->getSizeInCellsX();
  size_y_ = costmap_ros->getSizeInCellsY();
  RCLCPP_INFO(
    node_->get_logger(), "origin x %f y %f, resolution: %f,  size of x %d y %d",
    origin_x_, origin_y_, resolution_, size_x_, size_y_);
  
  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  RCLCPP_INFO(
    node_->get_logger(), "configure plugin %s of type NavfnPlanner",
    name_.c_str());
  std::string filename = "/data/path.txt";
  poses = readPathsFromFile(filename);
  std::vector<std::vector<int>> graph = convertPosesToGridMap(poses, size_y_, size_x_);
}

std::vector<std::vector<int>> DefinedWaypoints::convertPosesToGridMap(const std::vector<Pose>& poses, int grid_width, int grid_height) {
  std::vector<std::vector<int>> grid_map(grid_height, std::vector<int>(grid_width, 0));
    std::vector<std::vector<Vec3b>> gridMap(grid_height, std::vector<Vec3b>(grid_width, Vec3b(255, 255, 255))); // Initialize with white color

    for (const auto& pose : poses) {
      unsigned int y_index = std::floor((pose.y - origin_y_) / resolution_);
      unsigned int x_index = std::floor((pose.x - origin_x_) / resolution_);
      grid_map[y_index][x_index] = 1;
      gridMap[y_index][x_index] = Vec3b(0, 0, 0);
      RCLCPP_INFO(
    node_->get_logger(), "pose x %f y %f, index x: %d,  y %d",
    pose.x, pose.y, x_index, y_index);
    }
    Mat img(grid_height, grid_width, CV_8UC3, reinterpret_cast<uchar*>(gridMap.data()));
    std::string filename = "/data/grid_map.jpg";
    imwrite(filename, img);
    return grid_map;
}

std::vector<Pose> DefinedWaypoints::readPathsFromFile(const std::string& filename){
  std::vector<Pose> poses;
    std::ifstream file(filename);
    if (file.is_open()) {
        double x, y;
        while (file >> x >> y) {
            poses.push_back({x, y});
        }
    }
    file.close();
    return poses;
}

bool DefinedWaypoints::isValid(int x, int y, const std::vector<std::vector<int>>& grid) {
    return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size();
}

std::vector<std::pair<int, int>> DefinedWaypoints::getNeighbors(int x, int y) {
    return {{x - 1, y}, {x + 1, y}, {x, y - 1}, {x, y + 1}};
}

int DefinedWaypoints::manhattanDistance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

std::vector<std::pair<int, int>> DefinedWaypoints::aStar(std::vector<std::vector<int>>& grid, std::pair<int, int> start, std::pair<int, int> end) {
    int rows = grid.size();
    int cols = grid[0].size();

    std::vector<std::vector<bool>> closedSet(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Node>> nodes(rows, std::vector<Node>(cols));

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            nodes[i][j].x = i;
            nodes[i][j].y = j;
            nodes[i][j].f = INT_MAX;
            nodes[i][j].g = INT_MAX;
            nodes[i][j].h = INT_MAX;
        }
    }

    nodes[start.first][start.second].g = 0;
    nodes[start.first][start.second].h = manhattanDistance(start.first, start.second, end.first, end.second);
    nodes[start.first][start.second].f = nodes[start.first][start.second].g + nodes[start.first][start.second].h;

    std::set<Node> openSet;
    openSet.insert(nodes[start.first][start.second]);

    while (!openSet.empty()) {
        Node current = *openSet.begin();
        openSet.erase(openSet.begin());

        if (current.x == end.first && current.y == end.second) {
            std::vector<std::pair<int, int>> path;
            while (current.x != start.first || current.y != start.second) {
                path.push_back({current.x, current.y});
                int x = current.x;
                int y = current.y;
                current = nodes[current.x][current.y];
            }
            path.push_back({start.first, start.second});
            std::reverse(path.begin(), path.end());
            return path;
        }

        closedSet[current.x][current.y] = true;

        for (const auto& neighbor : getNeighbors(current.x, current.y)) {
            int x = neighbor.first;
            int y = neighbor.second;
             if (!isValid(x, y, grid) || grid[x][y] == 1 || closedSet[x][y]) {
            continue;
        }
        int tentative_g = current.g + 1;

        if (tentative_g < nodes[x][y].g) {
            nodes[x][y].g = tentative_g;
            nodes[x][y].h = manhattanDistance(x, y, end.first, end.second);
            nodes[x][y].f = nodes[x][y].g + nodes[x][y].h;
            nodes[x][y] = current;

            bool inOpenSet = openSet.count(nodes[x][y]) > 0;

            if (!inOpenSet) {
                openSet.insert(nodes[x][y]);
            }
        }
    }
}

return {}; // No path found
}



void DefinedWaypoints::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void DefinedWaypoints::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void DefinedWaypoints::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path DefinedWaypoints::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (const auto& pose_struct : poses){
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = pose_struct.x;
    pose.pose.position.y = pose_struct.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  // global_path.poses.push_back(goal);

  return global_path;
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_definedwaypoints_planner::DefinedWaypoints, nav2_core::GlobalPlanner)
