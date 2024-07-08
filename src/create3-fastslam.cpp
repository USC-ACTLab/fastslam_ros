/**
 * @file create3-fastslam.cpp
 * @brief fastslam create3 wrapper. Used to collect sensor data from robot
 */
#include <chrono>
#include <functional>
#include <memory>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "robot-manager.h"
#include "particle-filter.h"

using namespace std::chrono_literals;

// constants for create3 pose sensor noise
static const float x_accel_var = 0.0001543f;
static const float y_accel_var = 0.0001636f;
static const float theta_var = 0.0000125f;


static float quaternion_to_yaw(
  const geometry_msgs::msg::Quaternion msg) {
    return atan2f(2*(msg.w * msg.z + msg.x*msg.y), 1 - 2*(msg.y * msg.y + msg.z * msg.z));
  }

class FastSLAMC3: public rclcpp::Node
{
public:
  FastSLAMC3(): Node("fastslam_create3"){
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&FastSLAMC3::odom_callback, this, std::placeholders::_1));
    m_landmark_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "landmark", 10, std::bind(&FastSLAMC3::lm_callback, this, std::placeholders::_1));
    
    const struct Pose2D init_pose {.x = 0, .y = 0, .theta_rad = 0};
    const struct VelocityCommand2D init_cmd {.vx_mps = 0, .wz_radps = 0};
    Eigen::Matrix3f rob_process_noise;
    rob_process_noise.diagonal() << x_accel_var, y_accel_var, theta_var;
    m_robot_manager = std::make_shared<Create3Manager>(init_pose, init_cmd, Eigen::Matrix2f::Zero(), 3.0f, rob_process_noise);
    m_fastslam_filter = std::make_unique<FastSLAMPF>(m_robot_manager);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  void odom_callback(const nav_msgs::msg::Odometry msg);
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_landmark_sub;
  void lm_callback(const sensor_msgs::msg::LaserScan msg);

  //FastSLAM member variables
  std::unique_ptr<FastSLAMPF> m_fastslam_filter;
  std::shared_ptr<Create3Manager> m_robot_manager;

  //robot parameters
  struct Pose2D m_rob_pose_mean;
  
};

void FastSLAMC3::odom_callback(const nav_msgs::msg::Odometry msg) {
  m_rob_pose_mean.theta_rad = quaternion_to_yaw(msg.pose.pose.orientation);
  RCLCPP_INFO(this->get_logger(), "Yaw: %f", m_rob_pose_mean.theta_rad);
  m_rob_pose_mean.x = msg.pose.pose.position.x;
  m_rob_pose_mean.y = msg.pose.pose.position.y;
}

void FastSLAMC3::lm_callback(const sensor_msgs::msg::LaserScan msg){
  RCLCPP_INFO(this->get_logger(), "I heard stuff");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastSLAMC3>());
  rclcpp::shutdown();
  return 0;
}