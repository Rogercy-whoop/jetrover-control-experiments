cmake_minimum_required(VERSION 3.8)
project(stage3_logger)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_robot_controller_msgs REQUIRED)

add_executable(step_logger src/step_logger.cpp)
ament_target_dependencies(step_logger rclcpp sensor_msgs ros_robot_controller_msgs)

add_executable(temp_logger src/temp_logger.cpp)
ament_target_dependencies(temp_logger rclcpp ros_robot_controller_msgs)

add_executable(heading_pid src/heading_pid.cpp)
ament_target_dependencies(heading_pid rclcpp geometry_msgs nav_msgs)

install(TARGETS step_logger temp_logger heading_pid DESTINATION lib/${PROJECT_NAME})
ament_package()
