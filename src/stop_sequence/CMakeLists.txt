cmake_minimum_required(VERSION 3.0.2)
project(stop_sequence)

find_package(catkin REQUIRED COMPONENTS
  rospy
  morai_msgs
  cv_bridge
  sensor_msgs
  vision_msgs
)

catkin_package()

install(DIRECTORY launch models config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/traffic_light_detection.py
  scripts/stop_line_detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
