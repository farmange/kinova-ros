cmake_minimum_required(VERSION 2.8.3)
project(orthopus_addon)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kinova_msgs
  roscpp
  std_msgs
  std_srvs
  joy
  rospy
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

############ device freejoy 
add_executable(device_freejoy
  src/input_device/device_freejoy.cpp
)
target_link_libraries(device_freejoy ${catkin_LIBRARIES})
add_dependencies(device_freejoy ${catkin_EXPORTED_TARGETS})


############ hl controller
add_executable(hl_controller
  src/hl_controller.cpp  
)
target_link_libraries(hl_controller ${catkin_LIBRARIES})
add_dependencies(hl_controller ${catkin_EXPORTED_TARGETS})
