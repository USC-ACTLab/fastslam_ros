
#include <memory>
#include <cmath>
#include <queue>
#include <vector>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "core-structs.h"
#include "lm-observation-and-visualization.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

static bool sameLandmark(const Observation2D &first_landmark, const Observation2D &second_landmark){
  float r1 = first_landmark.range_m;
  float r2 = second_landmark.range_m;
  float theta1 = first_landmark.bearing_rad;
  float theta2 = second_landmark.bearing_rad;
  float distance = sqrt(pow(r1,2.0)+pow(r2, 2.0)-2*r1*r2*cos(theta1-theta2));
  return  distance <= LM_THRESHOLD_m;
}

static void mergeLandmarks(Observation2D &existing_landmark, Observation2D &new_measurement, int &count){
  existing_landmark.range_m = ((count-1)*existing_landmark.range_m+new_measurement.range_m)/count;
  existing_landmark.bearing_rad = ((count-1)*existing_landmark.bearing_rad+new_measurement.bearing_rad)/count;
}

static void mergeLandmarks(Observation2D &first_landmark, Observation2D &last_landmark, const int &count1, const int &count2){
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

void visualizePFLandmarks(const std::vector<Point2D>& landmarks, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr landmark_visualization_publisher){
  auto message = visualization_msgs::msg::Marker();
  rclcpp::Clock clock = rclcpp::Clock();
  message.header.stamp = clock.now();
  message.header.frame_id = "map";
  message.action = 0;
  message.type = 8;
  message.ns = "landmark_list";
  message.id = 0;
  message.pose.orientation.w = 1.0;
  message.scale.x = LM_VISUALIZATION_SCALE_m;
  message.scale.y = LM_VISUALIZATION_SCALE_m;
  auto color = std_msgs::msg::ColorRGBA();
  color.r = LM_COLOR_RED;
  color.g = LM_COLOR_GREEN;
  color.b = LM_COLOR_RED;
  color.a = 1.0;
  for(const auto& curr_landmark: landmarks){
    auto curr_point = geometry_msgs::msg::Point();
    curr_point.x = curr_landmark.x;
    curr_point.y = curr_landmark.y;
    message.points.push_back(curr_point);
    message.colors.push_back(color);
  }
  landmark_visualization_publisher->publish(message);
}

std::queue<Observation2D> calculateLMObservations(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  float angle_increment = msg->angle_increment;
  int measurement_count = static_cast<int>(std::round((msg->angle_max-msg->angle_min)/angle_increment)) + 1;
  std::queue<Observation2D> landmarks;
  int first_count=0;
  int last_count=0;
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
      first_count=1;
      last_count=1; 	       
    }else if(sameLandmark(curr_measurement, previous_measurement)){
      if (landmarks.size()==1){
        first_count++; //if there is only one LM the last LM and first LM refer to the same LM and should both have their counts increased
      }
      last_count++;
      mergeLandmarks(landmarks.back(), curr_measurement, last_count);
    }else{
      landmarks.push(curr_measurement);
      //creating a new landmark sets the last count back to 1
      last_count = 1;
    }
    previous_measurement = curr_measurement;
  }
  struct Observation2D last_measurement = previous_measurement;
  //if the first_landmark and last landmarks are actually the same, they get combined!
  if (landmarks.size()>1 and sameLandmark(first_landmark_measurement, last_measurement)){
    mergeLandmarks(landmarks.front(), landmarks.back(), first_count, last_count);
    landmarks.pop();
  }
  return landmarks;
}

void visualizeLMObservations(std::queue<Observation2D>& landmarks, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher){
  auto message = visualization_msgs::msg::Marker();
  rclcpp::Clock clock = rclcpp::Clock();
  message.header.stamp = clock.now();
  message.header.frame_id = "laser";
  message.action = 0;
  message.type = 8;
  message.ns = "landmark_list";
  message.id = 0;
  message.pose.orientation.w = 1.0;
  message.scale.x = LM_VISUALIZATION_SCALE_m;
  message.scale.y = LM_VISUALIZATION_SCALE_m;
  auto color = std_msgs::msg::ColorRGBA();
  color.r = LM_COLOR_RED;
  color.g = LM_COLOR_GREEN;
  color.b = LM_COLOR_BLUE;
  color.a = 1.0;
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

std::queue<Observation2D> calculateAndVisualizeLMObservations(const sensor_msgs::msg::LaserScan::SharedPtr msg, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr observation_visualization_publisher){
  std::queue<Observation2D> landmarks = calculateLMObservations(msg);
  std::queue<Observation2D>landmarks_copy(landmarks);
  visualizeLMObservations(landmarks_copy, observation_visualization_publisher);
  return landmarks;
}
