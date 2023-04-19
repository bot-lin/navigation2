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
#include "nav2_util/node_utils.hpp"

#include "nav2_definedwaypoints_planner/defined_waypoints_planner.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "nav2_definedwaypoints_planner/stb_image_write.h"
#include <utility>
#include <queue>
#include <limits>

typedef std::pair<double, double> DoublePoint;
using BinaryImage = std::vector<std::vector<int>>;

void saveBinaryImageAsPNG(const std::vector<std::vector<int>>& binaryImage, const std::string& filename) {
    int width = binaryImage[0].size();
    int height = binaryImage.size();
    int channels = 1; // Grayscale image

    // Create a 1D vector to store the image data
    std::vector<unsigned char> imageData(width * height * channels);

    // Convert the binary image to grayscale
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            imageData[i * width + j] = binaryImage[i][j] ? 255 : 0;
        }
    }

    // Save the grayscale image as a PNG file
    stbi_write_png(filename.c_str(), width, height, channels, imageData.data(), width * channels);
}
double euclideanDistance(const DoublePoint& p1, const DoublePoint& p2) {
    return std::sqrt(std::pow(p1.first - p2.first, 2) + std::pow(p1.second - p2.second, 2));
}

DoublePoint findClosestPoint(const DoublePoint& target, const std::vector<Pose>& points) {
    double minDistance = std::numeric_limits<double>::max();
    DoublePoint closestPoint = {points[0].x, points[0].y};

    for (const auto& point : points) {
        double distance = euclideanDistance(target, {point.x,point.y});
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = {point.x, point.y};
        }
    }

    return closestPoint;
}

using namespace cv;
struct MapNode {
    int x, y;
    MapNode(int x, int y) : x(x), y(y) {}
};

const int dx[] = {-1, 1, 0, 0};
const int dy[] = {0, 0, -1, 1};
bool operator==(const MapNode& a, const MapNode& b) {
    return a.x == b.x && a.y == b.y;
}
bool isValid(int x, int y, int rows, int cols, const std::vector<std::vector<bool>>& visited) {
    return x >= 0 && x < rows && y >= 0 && y < cols && !visited[x][y];
}

std::vector<MapNode> bfs(std::vector<std::vector<int>>& grid, MapNode start, MapNode end) {
    int rows = grid.size();
    int cols = grid[0].size();

    std::queue<MapNode> q;
    std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<MapNode>> parent(rows, std::vector<MapNode>(cols, MapNode(-1, -1)));

    q.push(start);
    visited[start.x][start.y] = true;

    while (!q.empty()) {
        MapNode current = q.front();
        q.pop();

        if (current == end) {
            break;
        }

        for (int i = 0; i < 4; ++i) {
            int newX = current.x + dx[i];
            int newY = current.y + dy[i];

            if (isValid(newX, newY, rows, cols, visited) && grid[newX][newY] == 1) {
                q.push(MapNode(newX, newY));
                visited[newX][newY] = true;
                parent[newX][newY] = current;
            }
        }
    }

    std::vector<MapNode> path;
    if (!visited[end.x][end.y]) {
        return path;  // Empty path if end point not visited
    }

    // Reconstruct the path from the parent matrix
    MapNode current = end;
    while (!(current == start)) {
        path.push_back(current);
        current = parent[current.x][current.y];
    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());
    return path;
}


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
  graph_ = convertPosesToGridMap(poses, size_y_, size_x_);
}

std::vector<std::vector<int>> DefinedWaypoints::convertPosesToGridMap(const std::vector<Pose>& poses, int grid_width, int grid_height) {
  std::vector<std::vector<int>> grid_map(grid_height, std::vector<int>(grid_width, 0));
    std::vector<std::vector<Vec3b>> gridMap(grid_height, std::vector<Vec3b>(grid_width, Vec3b(255, 255, 255))); // Initialize with white color

    for (const auto& pose : poses) {
      unsigned int y_index = std::floor((pose.y - origin_y_) / resolution_);
      unsigned int x_index = std::floor((pose.x - origin_x_) / resolution_);
      grid_map[x_index][y_index] = 1;
      gridMap[x_index][y_index] = Vec3b(0, 0, 0);
      RCLCPP_INFO(
    node_->get_logger(), "pose x %f y %f, index x: %d,  y %d",
    pose.x, pose.y, x_index, y_index);
    }
    std::string filename = "/data/grid_map1.png";
    saveBinaryImageAsPNG(grid_map, filename);
    std::cout << "Image saved as " << filename << std::endl;
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
  DoublePoint closestPoint = findClosestPoint({start.pose.position.x, start.pose.position.y}, poses);
  RCLCPP_INFO(node_->get_logger(), "Closest point to start is: %f, %f", closestPoint.first, closestPoint.second);
  unsigned int start_y_index = std::floor((closestPoint.second - origin_y_) / resolution_);
  unsigned int start_x_index = std::floor((closestPoint.first - origin_x_) / resolution_);
  closestPoint = findClosestPoint({goal.pose.position.x, goal.pose.position.y}, poses);
  RCLCPP_INFO(node_->get_logger(), "Closest point to end is: %f, %f", closestPoint.first, closestPoint.second);

  unsigned int end_y_index = std::floor((closestPoint.second - origin_y_) / resolution_);
  unsigned int end_x_index = std::floor((closestPoint.first - origin_x_) / resolution_);
  MapNode start_node = MapNode(start_x_index, start_y_index);
  MapNode end_node = MapNode(end_x_index, end_y_index);
  std::vector<MapNode> shortest_path = bfs(graph_, start_node, end_node);
  for (const auto& point : shortest_path) {
        std::cout << "(" << point.x << ", " << point.y << ") ";
    }
  std::cout << std::endl;
  for (const auto& point : shortest_path) {
    float x = point.x * resolution_ + origin_x_;
    float y = point.y * resolution_ + origin_y_;
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
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
