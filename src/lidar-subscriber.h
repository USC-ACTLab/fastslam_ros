#include <queue>
#include "core-structs.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher);

