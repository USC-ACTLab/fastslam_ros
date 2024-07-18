
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

#include "core-structs.h"


#include "sensor_msgs/msg/laser_scan.hpp"
#include <queue>
#include <limits>

constexpr float LANDMARK_THRESHOLD_m = 0.0125;

bool same_landmark(const Observation2D &first, const Observation2D &second){
  float r1 = first.range_m;
  float r2 = second.range_m;
  float theta1 = first.bearing_rad;
  float theta2 = second.bearing_rad;
  float distance = sqrt(pow(r1,2.0)+pow(r2, 2.0)-2*r1*r2*cos(theta1-theta2));
  if (distance>= LANDMARK_THRESHOLD_m){
      std::cout<<distance<<" ";
  }

  return  distance <= LANDMARK_THRESHOLD_m;
}

Observation2D merge(const Observation2D &existing_landmark, const Observation2D &new_measurement, int count){
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
    }
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
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
       	return landmarks;
    }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}
