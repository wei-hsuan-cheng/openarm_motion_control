cmake_minimum_required(VERSION 3.8)
project(openarm_motion_control)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Suppress warnings for Eigen
add_compile_options(-Wno-class-memacces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)  # <-- for using actions in C++
find_package(rclcpp_components REQUIRED)  # <-- for using actions in C++
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# Dependencies list
set(dependencies
    ament_cmake
    rclcpp
    rclcpp_action
    rclcpp_components
    Eigen3
    tf2_ros
    sensor_msgs
    visualization_msgs
    geometry_msgs
)

# For ROS 2 action
ament_export_dependencies(rosidl_default_runtime)


# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Source files
set(POSE_VISUALIZATION_LEFT_SOURCES src/pose_visualization_left.cpp)
set(POSE_VISUALIZATION_RIGHT_SOURCES src/pose_visualization_right.cpp)
set(ROBOT_JOINT_DYNAMICS_SOURCES src/robot_joint_dynamics.cpp)
set(JOINT_SPACE_MOTION_CONTROL_SOURCES src/joint_space_motion_control.cpp)
set(TASK_SPACE_MOTION_CONTROL_SOURCES src/task_space_motion_control.cpp)
set(MOTION_REFERENCE_GENERATOR_SOURCES src/motion_reference_generator.cpp)

# Add executables
add_executable(pose_visualization_left ${POSE_VISUALIZATION_LEFT_SOURCES})
add_executable(pose_visualization_right ${POSE_VISUALIZATION_RIGHT_SOURCES})
add_executable(robot_joint_dynamics ${ROBOT_JOINT_DYNAMICS_SOURCES})
add_executable(joint_space_motion_control ${JOINT_SPACE_MOTION_CONTROL_SOURCES})
add_executable(task_space_motion_control ${TASK_SPACE_MOTION_CONTROL_SOURCES})
add_executable(motion_reference_generator ${MOTION_REFERENCE_GENERATOR_SOURCES})

# Link libraries
ament_target_dependencies(pose_visualization_left ${dependencies})
ament_target_dependencies(pose_visualization_right ${dependencies})
ament_target_dependencies(robot_joint_dynamics ${dependencies})
ament_target_dependencies(joint_space_motion_control ${dependencies})
ament_target_dependencies(task_space_motion_control ${dependencies})
ament_target_dependencies(motion_reference_generator ${dependencies})

# Install targets
install(TARGETS
  pose_visualization_left
  pose_visualization_right
  robot_joint_dynamics
  joint_space_motion_control
  task_space_motion_control
  motion_reference_generator

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(
  DIRECTORY launch screw_lists rviz config
  DESTINATION share/${PROJECT_NAME}
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
