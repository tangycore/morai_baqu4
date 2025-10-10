cmake_minimum_required(VERSION 3.0.2)
project(planning_pkg)

# --- Python setup for ROS ---
catkin_python_setup()

# --- Required packages ---
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

# --- Message definitions ---
add_message_files(
  FILES
  PlanVelocityInfo.msg
)

# --- Generate message code ---
generate_messages(
  DEPENDENCIES std_msgs
)

# --- Catkin package configuration ---
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs message_runtime
)

# --- Include directories ---
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# --- Python executable installation ---
catkin_install_python(PROGRAMS
  scripts/global_path.py
  scripts/local_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
