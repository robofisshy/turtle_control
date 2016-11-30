cmake_minimum_required(VERSION 2.8.3)
project(turtle_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
)



catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_control src/main.cpp src/Turtle_control.cpp)

target_link_libraries(turtle_control
  ${catkin_LIBRARIES}
 )


