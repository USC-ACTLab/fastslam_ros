/**
 * @file lidar-subscriber.h
 * @brief Processes and sisualizes LiDAR readings
 */

#include <memory>
#include <cmath>
#include <queue>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "core-structs.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

/**
 * @brief Publishes visualization messages for landmarks in the global frame
 * @details Intended to be used with sampleLandmarks() in FastSLAMPF
 * @param[in] landmarks: queue of landmarks to be visualized
 * @param[in] landmark_visualization_publisher: publisher to publish the visualization messages
 */

void visualize_landmarks(std::queue<Point2D> landmarks, 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_visualization_publisher);

/**
 * @brief Clusters LiDAR data into landmark observations
 * @param[in] msg: LiDAR LaserScan to be processed
 * @return a queue of Observation2Ds corresponding to the observed landmarks
 */
std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg);

/**
 * @overload std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg);
 * @brief Clusters LiDAR data into landmark observations, publishes information needed to visualize landmarks in laser frame
 * @details VISUALIZE_LANDMARK_OBSERVATIONS flag needs to be turned on
 * @param[in] msg: LiDAR LaserScan to be processed
 * @param[in] observation_visualization_publisher: publisher to publish the visualization messages
 * @return a queue of Observation2Ds corresponding to the observed landmarks
 */
std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg, 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher);