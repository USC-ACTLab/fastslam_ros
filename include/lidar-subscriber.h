#include <memory>
#include <cmath>
#include <queue>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "core-structs.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
void visualize_landmarks(std::queue<Observation2D> landmarks, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher, std::string frame_id);

std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher);
