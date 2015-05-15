cmake_minimum_required(VERSION 2.8.3)
project(coro_tray_exekutor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  action_exekutor
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES coro_tray_exekutor
  CATKIN_DEPENDS action_exekutor
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(coro_tray_exekutor
  src/coro_tray_exekutor.cpp
)