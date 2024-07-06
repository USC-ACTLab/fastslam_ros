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

add_executable(publisher_member_function member_function.cpp)
target_link_libraries(publisher_member_function FastSLAMLib)
ament_target_dependencies(publisher_member_function rclcpp std_msgs)

add_executable(create3 src/create3-fastslam.cpp)
target_link_libraries(create3 FastSLAMLib)
ament_target_dependencies(create3 rclcpp std_msgs irobot_create_msgs 
                          nav_msgs sensor_msgs)

install(TARGETS
  publisher_member_function
  create3
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
