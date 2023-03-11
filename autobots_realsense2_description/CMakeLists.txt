cmake_minimum_required(VERSION 3.0.2)
project(autobots_realsense2_description)

find_package(catkin REQUIRED COMPONENTS)
catkin_package(CATKIN_DEPENDS)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
