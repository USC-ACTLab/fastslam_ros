cmake_minimum_required(VERSION 3.5)
project(slam_publisher)

# Add the FastSLAM directory
add_subdirectory(FastSLAM)
add_library(LidarSubscriberLib src/lidar_subscriber.cpp)
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

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
target_include_directories(LidarSubscriberLib PRIVATE ${PROJECT_SOURCE_DIR}/include)
add_executable(member_function src/member_function.cpp)
add_executable(create3 src/create3-fastslam.cpp)

# Link dependencies

ament_target_dependencies(member_function rclcpp std_msgs)
ament_target_dependencies(LidarSubscriberLib rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs)
ament_target_dependencies(create3 rclcpp std_msgs irobot_create_msgs nav_msgs sensor_msgs visualization_msgs)

# Optionally define compilation flags
option(USE_ROB_ODOM "Use robot odometry estimate" ON)
option(USE_ROB_IMU "Use robot imu for estimate" OFF)
option(USE_ROB_CTRL "Use robot control for estimate" OFF)

# Link libraries for create3
target_link_libraries(member_function FastSLAMLib)
target_link_libraries(LidarSubscriberLib FastSLAMLib)
target_link_libraries(create3 FastSLAMLib LidarSubscriberLib)
target_compile_definitions(create3 PUBLIC USE_ROB_ODOM USE_ROB_IMU USE_ROB_CTRL)

# Install targets
install(TARGETS
  member_function
  create3
  LidarSubscriberLib
  DESTINATION lib/${PROJECT_NAME}
)

# Test dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
