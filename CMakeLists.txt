cmake_minimum_required(VERSION 3.5)
project(slam_publisher)

# Add the FastSLAM directory
add_subdirectory(FastSLAM)
add_library(LMLib src/lm-observation-and-visualization.cpp)
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
target_include_directories(LMLib PRIVATE ${PROJECT_SOURCE_DIR}/include)
add_executable(member_function src/member_function.cpp)
add_executable(create3 src/create3-fastslam.cpp)
add_executable(vicon_visualization src/vicon-visualization.cpp)

# Link dependencies

ament_target_dependencies(member_function rclcpp std_msgs)
ament_target_dependencies(LMLib rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs)
ament_target_dependencies(create3 rclcpp std_msgs irobot_create_msgs nav_msgs sensor_msgs visualization_msgs)
ament_target_dependencies(vicon_visualization rclcpp std_msgs geometry_msgs visualization_msgs)

# Optionally define compilation flags
option(USE_ROB_ODOM "Use robot odometry estimate" ON)
option(USE_ROB_IMU "Use robot imu for estimate" OFF)
option(USE_ROB_CTRL "Use robot control for estimate" OFF)
option (VISUALIZE_LANDMARK_OBSERVATIONS "publishes information to visualize landmark observations in laser frame" OFF)
option (VISUALIZE_SLAM "publishes information to visualize SLAM LM estimates and robot path in map frame" OFF)
option (VISUALIZE_VICON "publishes information necessary to visualize ground truth LMs and robot path" OFF)

# Link libraries for create3
target_link_libraries(member_function FastSLAMLib)
target_link_libraries(LMLib FastSLAMLib)
target_link_libraries(create3 FastSLAMLib LMLib)
target_compile_definitions(create3 PUBLIC USE_ROB_ODOM USE_ROB_IMU USE_ROB_CTRL VISUALIZE_SLAM VISUALIZE_LANDMARK_OBSERVATIONS VISUALIZE VISUALIZE_VICON)
# Install targets
install(TARGETS
  member_function
  create3
  LMLib
  vicon_visualization
  DESTINATION lib/${PROJECT_NAME}
)

# Test dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
