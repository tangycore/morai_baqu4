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
catkin_python_setup()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
