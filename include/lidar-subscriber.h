#include <memory>
#include <cmath>
#include <queue>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "core-structs.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"


//bool same_landmark(const Observation2D &first_landmark, const Observation2D &second_landmark);

//void merge_landmarks(Observation2D &existing_landmark, Observation2D &new_measurement, int &count);

//void merge_landmarks(Observation2D &first_landmark, Observation2D &last_landmark, const int &count1, const int &count2);

//void visualize_landmarks(std::queue<Observation2D> landmarks, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher);

std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher);