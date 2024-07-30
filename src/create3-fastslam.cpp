/**
 * @file create3-fastslam.cpp
 * @brief fastslam create3 wrapper. Used to collect sensor data from robot
 */
#include <chrono>
#include <functional>
#include <memory>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "irobot_create_msgs/msg/wheel_vels.hpp"
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

constexpr float IMU_UPDATE_T_s = 1.0f / 100.0f;
constexpr float IMU_INTEGRATION_s2 = IMU_UPDATE_T_s * IMU_UPDATE_T_s;

#ifdef USE_ROB_IMU
static float vx = 0.0f;
static float vy = 0.0f;
static float x_pos = 0.0f;
static float y_pos = 0.0f;
#endif // USE_ROB_IMU


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
#elif defined(USE_ROB_IMU)
    rmw_qos_profile_t imu_init_profile = rmw_qos_profile_default;
    auto imu_qos_profile = std::make_unique<rclcpp::QoS>(rclcpp::QoSInitialization(imu_init_profile.history, 10), imu_init_profile);
    imu_qos_profile->best_effort();
    imu_qos_profile->durability_volatile();
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "robot_1/imu", *imu_qos_profile, std::bind(&FastSLAMC3::imu_callback, this, std::placeholders::_1));
#else // use robot control signal
    m_control_sub = this->create_subscription<irobot_create_msgs::msg::WheelVels>(
      "robot_1/wheel_vels", 10, std::bind(&FastSLAMC3::control_callback, this, std::placeholders::_1));
#endif // Motion model selection

    m_landmark_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 10, std::bind(&FastSLAMC3::lm_callback, this, std::placeholders::_1));
    m_deadreckon_odom = this->create_publisher<nav_msgs::msg::Odometry>("slam_motion", 10);
    
    const struct Pose2D init_pose {.x = 0, .y = 0, .theta_rad = 0};
    const struct VelocityCommand2D init_cmd {.vx_mps = 0, .wz_radps = 0};
    Eigen::Matrix3f rob_process_noise;
    rob_process_noise.diagonal() << x_accel_var, y_accel_var, theta_var;

    m_robot_manager = std::make_shared<Create3Manager>(init_pose, init_cmd, Eigen::Matrix2f::Zero(), 3.0f, rob_process_noise);
    m_fastslam_filter = std::make_unique<FastSLAMPF>(
      std::static_pointer_cast<RobotManager2D>(m_robot_manager));

    m_rob_pose_delta = {.x = 0, .y = 0, .theta_rad = 0};
    m_prev_rob_pose = m_rob_pose_delta;
    m_pose_calibrated = false;
  }

private:
#ifdef USE_ROB_ODOM
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
  void odom_callback(const nav_msgs::msg::Odometry& msg);
  struct Pose2D m_prev_rob_pose;
#elif defined(USE_ROB_IMU)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
  void imu_callback(const sensor_msgs::msg::Imu& msg);
#else
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_control_sub;
  void control_callback(const sensor_msgs::msg::Imu& msg);
#endif //Motion model selection
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_landmark_sub;
  void lm_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_deadreckon_odom;
  void publishSLAMOdom();

  //FastSLAM member variables
  std::unique_ptr<FastSLAMPF> m_fastslam_filter;
  std::shared_ptr<Create3Manager> m_robot_manager;

  //robot parameters
  struct Pose2D m_rob_pose_delta;
  bool m_pose_calibrated; // discard first sample
  
};

#ifdef USE_ROB_ODOM
void FastSLAMC3::odom_callback(const nav_msgs::msg::Odometry& msg) {
  const struct Pose2D curr_pose = {.x = msg.pose.pose.position.x,
                                   .y = msg.pose.pose.position.y,
                                   .theta_rad = quaternion_to_yaw(msg.pose.pose.orientation)};
  auto crude_angle = curr_pose.theta_rad - m_prev_rob_pose.theta_rad;

  if (m_pose_calibrated) {
    m_rob_pose_delta.x = curr_pose.x - m_prev_rob_pose.x;
    m_rob_pose_delta.y = curr_pose.y - m_prev_rob_pose.y;
    // angle wrap around
    m_rob_pose_delta.theta_rad = crude_angle - 2*M_PI*floorf((crude_angle + M_PI) / 2*M_PI);
  } else {
    m_pose_calibrated = true;
  }
  m_prev_rob_pose = curr_pose;

  this->publishSLAMOdom();
} // USE_ROB_ODOM
#elif defined USE_ROB_IMU
void FastSLAMC3::imu_callback(const sensor_msgs::msg::Imu& msg){
  RCLCPP_INFO(this->get_logger(), "Publishing robot odom delta from IMU");
  auto a_x = msg.linear_acceleration.x;
  auto a_y = msg.linear_acceleration.y;

  m_rob_pose_delta.x = vx * IMU_UPDATE_T_s + a_x * IMU_INTEGRATION_s2 * 0.5;
  m_rob_pose_delta.y = vy * IMU_UPDATE_T_s + a_y * IMU_INTEGRATION_s2 * 0.5;
  vx += a_x * IMU_UPDATE_T_s;
  vy += a_y * IMU_UPDATE_T_s;

  this->publishSLAMOdom();

} //USE_ROB_IMU
#else
void FastSLAMC3::control_callback(const irobot_create_msgs::msg::WheelVels& msg){
}
#endif //Motion model selection

void FastSLAMC3::publishSLAMOdom() {
  RCLCPP_INFO(this->get_logger(), "Publishing robot odom delta");
  auto message = nav_msgs::msg::Odometry();
  message.pose.pose.position.x = m_rob_pose_delta.x;
  message.pose.pose.position.y = m_rob_pose_delta.y;
  x_pos += m_rob_pose_delta.x;
  y_pos += m_rob_pose_delta.y;
  RCLCPP_INFO(this->get_logger(), "Accu: x %f y %f", x_pos, y_pos);

  m_deadreckon_odom->publish(message);
}

void FastSLAMC3::lm_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

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
