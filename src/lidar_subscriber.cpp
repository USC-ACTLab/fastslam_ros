
#include <memory>
#include <cmath>
#include <queue>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "core-structs.h"
#include "lidar-subscriber.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

constexpr float LANDMARK_THRESHOLD_m = 0.02;
#define VISUALIZE_LANDMARK_OBSERVATIONS

static bool same_landmark(const Observation2D &first_landmark, const Observation2D &second_landmark){
  float r1 = first_landmark.range_m;
  float r2 = second_landmark.range_m;
  float theta1 = first_landmark.bearing_rad;
  float theta2 = second_landmark.bearing_rad;
  float distance = sqrt(pow(r1,2.0)+pow(r2, 2.0)-2*r1*r2*cos(theta1-theta2));
  return  distance <= LANDMARK_THRESHOLD_m;
}

static void merge_landmarks(Observation2D &existing_landmark, Observation2D &new_measurement, int &count){
  existing_landmark.range_m = ((count-1)*existing_landmark.range_m+new_measurement.range_m)/count;
  existing_landmark.bearing_rad = ((count-1)*existing_landmark.bearing_rad+new_measurement.bearing_rad)/count;
}

static void merge_landmarks(Observation2D &first_landmark, Observation2D &last_landmark, const int &count1, const int &count2){
  //standard weighted average --> between first_landmark and last landmark
  float weight1 = ((float)count1)/((float)(count1+count2));
  float weight2 = ((float)count2)/((float)(count1+count2));
  if(first_landmark.bearing_rad>M_PI){
          first_landmark.bearing_rad -=2*M_PI;
  }
  if(last_landmark.bearing_rad>M_PI){
          last_landmark.bearing_rad-=2*M_PI;
  }
  last_landmark.range_m = first_landmark.range_m*weight1+last_landmark.range_m*weight2;
  last_landmark.bearing_rad = first_landmark.bearing_rad*weight1+(last_landmark.bearing_rad)*weight2;
}

void visualize_landmarks(std::queue<Observation2D> landmarks, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher){
  auto message = visualization_msgs::msg::Marker();
  rclcpp::Clock clock = rclcpp::Clock();
  message.header.stamp = clock.now();
  message.header.frame_id = "laser";
  message.action = 0;
  message.type = 8;
  message.ns = "landmark_list";
  message.id = 0;
  message.pose.orientation.w = 1.0;
  message.scale.x = 0.03;
  message.scale.y = 0.03;
  auto color = std_msgs::msg::ColorRGBA();
  color.r = 255;
  color.g = 255;
  color.b = 255;
  color.a = 0.8;
  while(!landmarks.empty()){
    auto curr_point = geometry_msgs::msg::Point();
    Observation2D curr_landmark = landmarks.front();
    curr_point.x = -1*curr_landmark.range_m*cos(curr_landmark.bearing_rad);
    curr_point.y = -1*curr_landmark.range_m*sin(curr_landmark.bearing_rad);
    message.points.push_back(curr_point);
    landmarks.pop();
    message.colors.push_back(color);
  }
  observation_visualization_publisher->publish(message);
}

std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher){
  float angle_increment = msg->angle_increment;
  int measurement_count = static_cast<int>(std::round((msg->angle_max-msg->angle_min)/angle_increment)) + 1;
  std::queue<Observation2D> landmarks;
  std::queue<int> counts; 
  struct Observation2D previous_measurement;
  struct Observation2D first_landmark_measurement;
  for (int i = 1; i < measurement_count; i++){
    float curr_range = msg->ranges[i];
    if (curr_range == std::numeric_limits<float>::infinity()||curr_range == -1*std::numeric_limits<float>::infinity()){ 
      continue;
    }
    float curr_bearing = i * angle_increment;
    struct Observation2D curr_measurement = {curr_range, curr_bearing};
    if (landmarks.empty()){
      landmarks.push(curr_measurement);
      previous_measurement = curr_measurement;
      first_landmark_measurement = curr_measurement;
      counts.push(1);	       
    }else if(same_landmark(curr_measurement, previous_measurement)){
      counts.back()++;
      merge_landmarks(landmarks.back(), curr_measurement, counts.back());
    }else{
      landmarks.push(curr_measurement);
      counts.push(1);
    }
    previous_measurement = curr_measurement;
  }
  struct Observation2D last_measurement = previous_measurement;
  //if the first_landmark and last landmarks are actually the same, they get combined!
  if (landmarks.size()>1 and same_landmark(first_landmark_measurement, last_measurement)){
    merge_landmarks(landmarks.front(), landmarks.back(), counts.front(), counts.back());
    landmarks.pop();
  }
  struct Observation2D curr_landmark;
  std::queue<Observation2D>wrapped_landmarks;
  while(!landmarks.empty()){
    curr_landmark = landmarks.front();
    if(curr_landmark.bearing_rad>M_PI){
      curr_landmark.bearing_rad -= 2*M_PI;
    }
    wrapped_landmarks.push(curr_landmark);
    landmarks.pop();
  }
#ifdef VISUALIZE_LANDMARK_OBSERVATIONS
  std::queue<Observation2D>landmarks_visual_copy(wrapped_landmarks);
  visualize_landmarks(landmarks_visual_copy, observation_visualization_publisher);
#endif 
  return wrapped_landmarks;
}
