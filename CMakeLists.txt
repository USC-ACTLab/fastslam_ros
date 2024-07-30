cmake_minimum_required(VERSION 3.5)
project(slam_publisher)

add_subdirectory(FastSLAM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # generate compile_commands.json


# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(publisher_member_function src/member_function.cpp)
add_executable(lidar_subscriber src/lidar_subscriber.cpp)
target_link_libraries(publisher_member_function FastSLAMLib)
target_link_libraries(lidar_subscriber FastSLAMLib)
ament_target_dependencies(publisher_member_function rclcpp std_msgs)
ament_target_dependencies(lidar_subscriber rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs)

option (USE_ROB_ODOM "Use robot odometry estimate" ON)
option (USE_ROB_IMU "Use robot imu for estimate" OFF)
option (USE_ROB_CTRL "Use robot control for estimate" OFF)
add_executable(create3 src/create3-fastslam.cpp)
target_link_libraries(create3 FastSLAMLib)
target_compile_definitions (create3 PUBLIC USE_ROB_ODOM USE_ROB_IMU USE_ROB_CTRL)
ament_target_dependencies(create3 rclcpp std_msgs irobot_create_msgs 
                          nav_msgs sensor_msgs)

install(TARGETS
  publisher_member_function
  create3
  lidar_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
