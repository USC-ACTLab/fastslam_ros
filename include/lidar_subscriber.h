#include <queue>
#include "core-structs.h"

std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher);

