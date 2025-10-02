cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(catkin REQUIRED COMPONENTS
 rospy
 std_msgs
 geometry_msgs
 nav_msgs
 morai_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/global_path_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# launch 설치(선택이지만 권장)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
