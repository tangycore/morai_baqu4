cmake_minimum_required(VERSION 3.0.2)
project(baqu4_tf)
find_package(catkin REQUIRED)
catkin_package()
# launch 설치(선택이지만 권장)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
