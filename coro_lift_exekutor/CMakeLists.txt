cmake_minimum_required(VERSION 2.8.3)
project(coro_lift_exekutor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  action_exekutor
  sensor_msgs
  cam_interface
  move_base_msgs
  actionlib
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES coro_lift_exekutor
  CATKIN_DEPENDS action_exekutor sensor_msgs
)

include_directories(include)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(coro_lift_exekutor
  src/coro_lift_exekutor.cpp
)

add_executable(coro_lift_exekutor_node src/lift_exek_test.cc)

add_dependencies(coro_lift_exekutor laser_lines_generated_messages_cpp)

target_link_libraries(coro_lift_exekutor
  ${catkin_LIBRARIES}
)

target_link_libraries(coro_lift_exekutor_node
  coro_lift_exekutor
  ${catkin_LIBRARIES}
)