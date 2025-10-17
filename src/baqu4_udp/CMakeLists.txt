cmake_minimum_required(VERSION 3.0.2)
project(baqu4_udp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  morai_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/CollisionDataNode.py
  scripts/VehicleStatusNode.py
  scripts/EgoCtrlCmdNode.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)