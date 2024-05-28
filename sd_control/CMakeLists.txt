cmake_minimum_required(VERSION 3.5)
project(sd_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sd_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_dev REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sd_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Install launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.yaml" PATTERN "*.urdf"
)

# Install config files
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        FILES_MATCHING PATTERN "*.yaml"
)

# Install scripts
install(PROGRAMS
        src/sd_teleop_keyboard.py
        DESTINATION lib/${PROJECT_NAME}
)

# Add library
add_library(SdControlPlugin SHARED src/sd_control_plugin.cpp)
ament_target_dependencies(SdControlPlugin gazebo_dev rclcpp sensor_msgs sd_msgs)  # Added sd_msgs

# Add executable
add_executable(sd_teleop_joy src/sd_teleop_joy.cpp src/sd_teleop_joy_node.cpp)
ament_target_dependencies(sd_teleop_joy rclcpp sensor_msgs sd_msgs)  # Added sd_msgs

# Ensure linking against rclcpp, sensor_msgs, and sd_msgs
target_link_libraries(sd_teleop_joy ${rclcpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${sd_msgs_LIBRARIES})

# Install targets
install(TARGETS SdControlPlugin sd_teleop_joy
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

