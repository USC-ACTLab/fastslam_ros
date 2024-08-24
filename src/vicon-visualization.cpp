//vicon callback functions


//make a parameter in launch file for number of landmarks
//subscriber to create3
//make a subscriber for each landmark --> naming convention?
//publish odom visualization information (follow rob path procedure)

//each LM can have the same callback function (publish visualization information)
//include RME stuff

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


using namespace std::chrono_literals;

class ViconVisualizer : public rclcpp::Node
{
public:
  ViconVisualizer()
  : Node("vicon_visualizer")
  {
    this->declare_parameter<int>("number_of_landmarks", int(0));

    if (this->get_parameter("number_of_landmarks").as_int() < 0){
      throw std::invalid_argument ("Number of landmarks is negative! Please edit launch file!");
    }
    m_vicon_path_pub = this->create_publisher<visualization_msgs::msg::Marker>("vicon_path_visualization", 10);
    m_vicon_path_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vicon/create3/create3/pose", 10, std::bind(&ViconVisualizer::create3_callback, this, std::placeholders::_1));

    for (int i = 0; i<this->get_parameter("number_of_landmarks").as_int(); i++){
      std::string sub_topic_name = "/vicon/landmark"+ std::to_string(i+1)+"/landmark"+std::to_string(i+1)+"/pose";
      //if that topic doesn't exist throw an error!
      std::string pub_topic_name ="vicon_landmark"+std::to_string(i+1)+"_visualization";
    m_vicon_landmark_subs[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        sub_topic_name, 10, [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            // Call the member function with both the message and `i`
            this->landmark_callback(*msg, i);  
        });
      m_vicon_landmark_pubs[i] = this->create_publisher<visualization_msgs::msg::Marker>(sub_topic_name, 10);
    }
    
  }

private:
  void create3_callback(const geometry_msgs::msg::PoseStamped& msg);
  void landmark_callback(const geometry_msgs::msg::PoseStamped& msg, int landmark_number);
  std::vector<geometry_msgs::msg::Point> m_path_points;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_vicon_path_sub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_vicon_path_pub;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> m_vicon_landmark_subs;
  std::vector <rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> m_vicon_landmark_pubs;

};

void ViconVisualizer::create3_callback(const geometry_msgs::msg::PoseStamped& msg){
  auto visualization_message = visualization_msgs::msg::Marker();
  visualization_message.header.frame_id = "map";
  visualization_message.ns = "path";
  visualization_message.id = 0;
  visualization_message.type = 4;
  auto curr_point = geometry_msgs::msg::Point();
  curr_point.x = msg.pose.position.y;
  curr_point.y = msg.pose.position.x;
  m_path_points.push_back(curr_point);
  visualization_message.points = m_path_points;	  
  visualization_message.color.g = 1.0;
  visualization_message.color.a = 1.0;
  visualization_message.scale.x = 0.02;
  this->m_vicon_path_pub->publish(visualization_message);
}

void ViconVisualizer::landmark_callback(const geometry_msgs::msg::PoseStamped& msg, int landmark_number){
  //Emma!
  //m_vicon_landmark_pubs[landmark_number]->publish(visualization_message)
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ViconVisualizer>());
  rclcpp::shutdown();
  return 0;
}
