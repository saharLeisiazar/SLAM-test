cmake_minimum_required(VERSION 2.8.3)
project(slam_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(goals src/goals.cpp)
target_link_libraries(goals ${catkin_LIBRARIES})
add_dependencies(goals slam_test)

catkin_install_python(PROGRAMS scripts/keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
