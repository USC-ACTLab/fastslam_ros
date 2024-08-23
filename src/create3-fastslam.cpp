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
#include "visualization_msgs/msg/marker.hpp"

#include "robot-manager.h"
#include "particle-filter.h"
#include "lm-observation-and-visualization.h"
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
    auto subscriber_qos = std::make_unique<rclcpp::QoS>(10);
    subscriber_qos->best_effort();
#ifdef USE_ROB_ODOM
    m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", *subscriber_qos, std::bind(&FastSLAMC3::odom_callback, this, std::placeholders::_1));
#elif defined(USE_ROB_IMU)
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", *subscriber_qos, std::bind(&FastSLAMC3::imu_callback, this, std::placeholders::_1));
    m_rob_pose= {.x = 0, .y = 0, .theta_rad = 0};
#else // use robot control signal
    m_control_sub = this->create_subscription<irobot_create_msgs::msg::WheelVels>(
      "/wheel_vels", *subscriber_qos, std::bind(&FastSLAMC3::control_callback, this, std::placeholders::_1));
#endif // Motion model selection
    m_landmark_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_filtered", 10, std::bind(&FastSLAMC3::lm_callback, this, std::placeholders::_1));
#ifdef VISUALIZE_LANDMARK_OBSERVATIONS
    m_observation_visualization_pub = this->create_publisher<visualization_msgs::msg::Marker>("landmark_observations",10);
#endif
    const struct Pose2D init_pose = {.x = 0, .y = 0, .theta_rad = 0};
    const struct VelocityCommand2D init_cmd {.vx_mps = 0, .wz_radps = 0};
    Eigen::Matrix3f rob_process_noise;
    rob_process_noise.diagonal() << x_accel_var, y_accel_var, theta_var;
    m_robot_manager = std::make_shared<Create3Manager>(init_pose, init_cmd, Eigen::Matrix2f::Zero(), 3.0f, rob_process_noise);
    m_fastslam_filter = std::make_unique<FastSLAMPF>(
    std::static_pointer_cast<RobotManager2D>(m_robot_manager));
#ifdef VISUALIZE_SLAM
    m_path_visualization_pub = this->create_publisher<visualization_msgs::msg::Marker>("path_visualization", 10);
    m_pf_landmarks_visualization_pub = this->create_publisher<visualization_msgs::msg::Marker>("pf_landmarks",10);
#endif 
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
#ifdef VISUALIZE_LANDMARK_OBSERVATIONS
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_observation_visualization_pub;
#endif
#ifdef VISUALIZE_SLAM
  std::vector<geometry_msgs::msg::Point> m_path_points;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_path_visualization_pub;
  void visualizeSLAMOdom();
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pf_landmarks_visualization_pub;
#endif

  //FastSLAM member variables
  std::unique_ptr<FastSLAMPF> m_fastslam_filter;
  std::shared_ptr<Create3Manager> m_robot_manager;
  //robot parameters
  struct Pose2D m_rob_pose; 
};

#ifdef USE_ROB_ODOM
void FastSLAMC3::odom_callback(const nav_msgs::msg::Odometry& msg) {
  m_rob_pose.x = msg.pose.pose.position.x;
  m_rob_pose.y = msg.pose.pose.position.y;
  m_rob_pose.theta_rad = quaternion_to_yaw(msg.pose.pose.orientation);
#ifdef VISUALIZE_ROB_PATH
  this->visualizeSLAMOdom();
#endif
} // USE_ROB_ODOM

#elif defined USE_ROB_IMU
void FastSLAMC3::imu_callback(const sensor_msgs::msg::Imu& msg){
  RCLCPP_INFO(this->get_logger(), "Publishing robot odom delta from IMU");
  auto a_x = msg.linear_acceleration.x;
  auto a_y = msg.linear_acceleration.y;
  m_rob_pose.x += vx * IMU_UPDATE_T_s + a_x * IMU_INTEGRATION_s2 * 0.5;
  m_rob_pose.y += vy * IMU_UPDATE_T_s + a_y * IMU_INTEGRATION_s2 * 0.5;
  vx += a_x * IMU_UPDATE_T_s;
  vy += a_y * IMU_UPDATE_T_s;
#ifdef VISUALIZE_ROB_PATH
  this->visualizeSLAMOdom();
#endif
} //USE_ROB_IMU

#else
void FastSLAMC3::control_callback(const irobot_create_msgs::msg::WheelVels& msg){
}
#endif //Motion model selection

#ifdef VISUALIZE_SLAM
void FastSLAMC3::visualizeSLAMOdom(){ 
  auto visualization_message = visualization_msgs::msg::Marker();
  visualization_message.header.frame_id = "map";
  visualization_message.ns = "path";
  visualization_message.id = 0;
  visualization_message.type = 4;
  auto curr_point = geometry_msgs::msg::Point();
  curr_point.x = m_rob_pose.x;
  curr_point.y = m_rob_pose.y;
  m_path_points.push_back(curr_point);
  visualization_message.points = m_path_points;	  
  visualization_message.color.b = 1.0;
  visualization_message.color.a = 1.0;
  visualization_message.scale.x = 0.02;
  this->m_path_visualization_pub->publish(visualization_message);
}
#endif

void FastSLAMC3::lm_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
#ifdef VISUALIZE_LANDMARK_OBSERVATIONS
  std::queue<Observation2D> lidar_landmarks = calculateAndVisualizeLMObservations(msg, m_observation_visualization_pub);
#else
  std::queue<Observation2D> lidar_landmarks = calculateLMObservations(msg);
#endif
  RCLCPP_INFO_ONCE(this->get_logger(), "Range: %f, bearing: %f", lidar_landmarks.front().range_m, lidar_landmarks.front().bearing_rad);
  m_fastslam_filter->updateFilter(m_rob_pose, lidar_landmarks);
#ifdef VISUALIZE_SLAM
  visualizePFLandmarks(m_fastslam_filter->sampleLandmarks(), m_pf_landmarks_visualization_pub);
#endif
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastSLAMC3>());
  rclcpp::shutdown();
  return 0;
}
