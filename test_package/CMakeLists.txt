cmake_minimum_required(VERSION 2.8.3)
project(test_package)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kinova_msgs
  roscpp
  rospy
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
