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
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "robot-manager.h"
#include "particle-filter.h"

using namespace std::chrono_literals;

// constants for create3 pose sensor noise
static const float x_accel_var = 0.0001543f;
static const float y_accel_var = 0.0001636f;
static const float theta_var = 0.0000125f;

constexpr float IMU_UPDATE_FREQ = 1.0f / 100.0f;


static float quaternion_to_yaw(
  const geometry_msgs::msg::Quaternion msg) {
    return atan2f(2*(msg.w * msg.z + msg.x*msg.y), 1 - 2*(msg.y * msg.y + msg.z * msg.z));
  }

class FastSLAMC3: public rclcpp::Node
{
public:
  FastSLAMC3(): Node("fastslam_create3"){
#ifdef USE_ROB_ODOM
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10, std::bind(&FastSLAMC3::odom_callback, this, std::placeholders::_1));
#endif
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&FastSLAMC3::imu_callback, this, std::placeholders::_1));
    m_landmark_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "landmark", 10, std::bind(&FastSLAMC3::lm_callback, this, std::placeholders::_1));

    m_deadreckon_odom = this->create_publisher<nav_msgs::msg::Odometry>("slam_motion", 10);
    
    const struct Pose2D init_pose {.x = 0, .y = 0, .theta_rad = 0};
    const struct VelocityCommand2D init_cmd {.vx_mps = 0, .wz_radps = 0};
    Eigen::Matrix3f rob_process_noise;
    rob_process_noise.diagonal() << x_accel_var, y_accel_var, theta_var;

    m_robot_manager = std::make_shared<Create3Manager>(init_pose, init_cmd, Eigen::Matrix2f::Zero(), 3.0f, rob_process_noise);
    m_fastslam_filter = std::make_unique<FastSLAMPF>(
      std::static_pointer_cast<RobotManager2D>(m_robot_manager));

    m_rob_pose_delta = {.x = 0, .y = 0, .theta_rad = 0};
  }

private:
#ifdef USE_ROB_ODOM
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  void odom_callback(const nav_msgs::msg::Odometry& msg);
#endif //USE_ROB_ODOM
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_landmark_sub;
  void lm_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  void imu_callback(const sensor_msgs::msg::Imu& msg);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_deadreckon_odom;

  //FastSLAM member variables
  std::unique_ptr<FastSLAMPF> m_fastslam_filter;
  std::shared_ptr<Create3Manager> m_robot_manager;

  //robot parameters
  struct Pose2D m_rob_pose_delta;
  
};

#ifdef USE_ROB_ODOM
void FastSLAMC3::odom_callback(const nav_msgs::msg::Odometry& msg) {
  m_rob_pose_delta.theta_rad = quaternion_to_yaw(msg.pose.pose.orientation);
  RCLCPP_INFO(this->get_logger(), "Yaw: %f", m_rob_pose_delta.theta_rad);
  m_rob_pose_delta.x = msg.pose.pose.position.x;
  m_rob_pose_delta.y = msg.pose.pose.position.y;
}
#endif //USE_ROB_ODOM

void FastSLAMC3::imu_callback(const sensor_msgs::msg::Imu& msg){
  auto a_x = msg.linear_acceleration.x;
  auto a_y = msg.linear_acceleration.y;
  
  m_rob_pose_delta.x += 0.5 * a_x  * IMU_UPDATE_FREQ;
  m_rob_pose_delta.x += 0.5 * a_y  * IMU_UPDATE_FREQ;
  m_rob_pose_delta.theta_rad = quaternion_to_yaw(msg.orientation);

  RCLCPP_DEBUG(this->get_logger(), "Publishing robot odom:");
  auto message = nav_msgs::msg::Odometry();
  message.pose.pose.position.x = m_rob_pose_delta.x;
  message.pose.pose.position.y = m_rob_pose_delta.y;

  m_deadreckon_odom->publish(message);
}

void FastSLAMC3::lm_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "I heard stuff");

#ifdef LASER_CODE_COMMITED
  std::queue<Observation2D> lidar_landmarks = laserscan_to_landmarks(msg);
  m_fastslam_filter->updateFilter(m_rob_pose_delta, lidar_landmarks);
#endif // LASER_CODE_COMMTIED
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastSLAMC3>());
  rclcpp::shutdown();
  return 0;
}
