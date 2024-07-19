
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

#include "core-structs.h"


#include "sensor_msgs/msg/laser_scan.hpp"
#include <queue>
#include <limits>
#include "visualization_msgs/msg/marker.hpp"

constexpr float LANDMARK_THRESHOLD_m = 0.0125;
constexpr bool testing = true;

bool same_landmark(const Observation2D &first, const Observation2D &second){
  float r1 = first.range_m;
  float r2 = second.range_m;
  float theta1 = first.bearing_rad;
  float theta2 = second.bearing_rad;
  float distance = sqrt(pow(r1,2.0)+pow(r2, 2.0)-2*r1*r2*cos(theta1-theta2));
  return  distance <= LANDMARK_THRESHOLD_m;
}

Observation2D merge(const Observation2D &existing_landmark, const Observation2D &new_measurement, int &count){
  struct Observation2D average;
  average.range_m = ((count-1)*existing_landmark.range_m+new_measurement.range_m)/count;
  average.bearing_rad = ((count-1)*existing_landmark.bearing_rad+new_measurement.bearing_rad)/count;
  return average;
}

Observation2D merge(const Observation2D &first, const Observation2D &second, const int &count1, const int &count2){
  //standard weighted average --> between first and last landmark
  struct Observation2D average;
  average.range_m = first.range_m*(count1/(count1+count2))+second.range_m*(count2/(count1+count2));
  average.bearing_rad = first.bearing_rad*(count1/(count1+count2))+second.bearing_rad*(count2/(count1+count2));
  return average;
}

class LidarSubscriber : public rclcpp::Node
{
  public:
    LidarSubscriber()
    : Node("lidar_subscriber")
    {
      subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_filtered", 10, std::bind(&LidarSubscriber::laserscan_to_landmarks, this, std::placeholders::_1));
      publisher = this->create_publisher<visualization_msgs::msg::Marker>("landmark_visualization",10);
    }
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher;
    std::queue<Observation2D> laserscan_to_landmarks(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        float angle_increment = msg->angle_increment;
        int measurement_count = static_cast<int>(std::round((msg->angle_max-msg->angle_min)/angle_increment)) + 1;
        std::queue<Observation2D> landmarks;
        std::queue<int> counts; 
	struct Observation2D previous_measurement;
        for (int i = 1; i < measurement_count; i++){
            float curr_range = msg->ranges[i];
            if (curr_range == std::numeric_limits<float>::infinity()||curr_range == -1*std::numeric_limits<float>::infinity()){ 
           continue;
	    }else{
                float curr_bearing = i * angle_increment;
		struct Observation2D curr_measurement = {curr_range, curr_bearing};
	       if (landmarks.empty()){
		       landmarks.push(curr_measurement);
		       previous_measurement = curr_measurement;
		       counts.push(1);	       
	       }else if(same_landmark(curr_measurement, previous_measurement)){
                  counts.back()++;
                  struct Observation2D updated_landmark = merge(landmarks.back(), curr_measurement, counts.back());
                  (landmarks.back()).range_m = updated_landmark.range_m;
                  (landmarks.back()).bearing_rad = updated_landmark.bearing_rad; 
                }else{
                  landmarks.push(curr_measurement);
                  counts.push(1);
                }
	       previous_measurement = curr_measurement;
            }
        }

        //if the first and last landmarks are actually the same, they get combined!
	if (landmarks.size()>1 and same_landmark(landmarks.front(), landmarks.back())){
          struct Observation2D new_last = merge(landmarks.front(), landmarks.back(), counts.front(), counts.back());
          (landmarks.back()).range_m = new_last.range_m;
          (landmarks.back()).bearing_rad = new_last.bearing_rad; 
          landmarks.pop();
        }
	
        std::cout<<landmarks.size()<<"\n";
	if (testing){
	    std::queue<Observation2D>landmarks_copy(landmarks);
	     visualize_landmarks(landmarks_copy);
	}
       	return landmarks;
    }

    void visualize_landmarks(std::queue<Observation2D> landmarks){
	 auto message = visualization_msgs::msg::Marker();
	 message.header.stamp = this->now();
	 message.header.frame_id = "laser";
	 message.action = 0;
	 message.type = 8;
	 message.ns = "landmark_list";
	 message.id = 0;
	 message.pose.orientation.w = 1.0;
	 message.scale.x = 0.2;
	 message.scale.y = 0.2;
	 message.color.r = 3;
	 message.color.g = 252;
	 message.color.b = 61;
	 while(!landmarks.empty()){
		auto curr_point = geometry_msgs::msg::Point();
		Observation2D curr_landmark = landmarks.front();
		curr_point.x = curr_landmark.range_m*cos(curr_landmark.bearing_rad);
		curr_point.y = curr_landmark.range_m*sin(curr_landmark.bearing_rad);
		message.points.push_back(curr_point);
		landmarks.pop();
	 }
	 publisher->publish(message);
    }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}