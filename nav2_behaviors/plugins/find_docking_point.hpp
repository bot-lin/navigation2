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
struct Point {
    double x, y;

    // Constructor to initialize the point
    Point(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}

    // Function to add two points
    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }

    // Function to subtract two points
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }

    // Function to scale a point by a factor
    Point operator*(double factor) const {
        return Point(x * factor, y * factor);
    }
};

// Function to calculate the distance between two points
double distance(const Point& A, const Point& B) {
    return std::sqrt(std::pow(B.x - A.x, 2) + std::pow(B.y - A.y, 2));
}

// Function to find the third point C on the line AB with a specified distance from B
Point findThirdPoint(const Point& A, const Point& B, double distBC) {
    Point AB = B - A; // Vector from A to B
    double distAB = distance(A, B);

    // Scale the AB vector to have the length of distBC
    Point scaledAB = AB * (distBC / distAB);

    // Calculate C as B plus the scaled AB vector
    Point C = B + scaledAB;
    return C;
}

struct Quaternion {
    double w, x, y, z;
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
};

std::pair<float, float> findIntersection(float xs1, float ys1, float xe1, float ye1, 
                                         float xs2, float ys2, float xe2, float ye2) {
    // Calculate slopes (m1, m2) and y-intercepts (c1, c2)
    float m1 = (ye1 - ys1) / (xe1 - xs1);
    float c1 = ys1 - m1 * xs1;
    float m2 = (ye2 - ys2) / (xe2 - xs2);
    float c2 = ys2 - m2 * xs2;

    if (m1 == m2) {
        std::cout << "Lines are parallel or coincident." << std::endl;
        return {0, 0}; // No intersection
    }

    float x = (c2 - c1) / (m1 - m2);
    float y = m1 * x + c1;

    return {x, y};
}

std::pair<double, double> findCircleLineIntersectionWithSmallerX(double x0, double y0, double r, 
                                                                 double xs, double ys, double xe, double ye) {
    double dx = xe - xs;
    double dy = ye - ys;
    double A = dx * dx + dy * dy;
    double B = 2 * (dx * (xs - x0) + dy * (ys - y0));
    double C = (xs - x0) * (xs - x0) + (ys - y0) * (ys - y0) - r * r;

    double det = B * B - 4 * A * C;

    if (A <= 0.0000001 || det < 0) {
        // No real solutions.
        throw std::runtime_error("No intersection or circle is tangent to the line.");
    } else if (det == 0) {
        // One solution.
        double t = -B / (2 * A);
        return {xs + t * dx, ys + t * dy};
    } else {
        // Two solutions.
        double t1 = (-B + std::sqrt(det)) / (2 * A);
        double t2 = (-B - std::sqrt(det)) / (2 * A);
        auto p1 = std::make_pair(xs + t1 * dx, ys + t1 * dy);
        auto p2 = std::make_pair(xs + t2 * dx, ys + t2 * dy);

        // Return the point with the smaller x value.
        return (p1.first < p2.first) ? p1 : p2;
    }
}

using FindDockingPointAction = nav2_msgs::action::FindDockingPoint;

class FindDockingPoint : public TimedBehavior<FindDockingPointAction>
{
public:
    FindDockingPoint();
    ~FindDockingPoint();
    Status onRun(const std::shared_ptr<const FindDockingPointAction::Goal> command) override;
    Status change_goal(const std::shared_ptr<const FindDockingPointAction::Goal> command) override;
    void onConfigure() override;
    Status onCycleUpdate() override;
    bool find_docking_spot();
    Point findClockwisePerpendicularVector(const Point& A, const Point& B);
    Quaternion vectorToQuaternion(const Point& vec);

   

protected:
    FindDockingPointAction::Feedback::SharedPtr feedback_;
    FindDockingPointAction::Result::SharedPtr result_;
    double distance_to_point_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> my_node_;
    rclcpp::Client<zbot_interfaces::srv::LineSegmentListSrv>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    bool processing_;
    bool do_reverse_;
    
};
}
#endif