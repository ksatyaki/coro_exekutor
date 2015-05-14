cmake_minimum_required(VERSION 2.8.3)
project(coro_exekutor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  coro_roller_exekutor
  load_map_exekutor
  miradock_exekutor
  move_to_exekutor
  cam_interface
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(coro_exekutor coro_exekutor_node.cpp)

target_link_libraries (coro_exekutor
    ${catkin_LIBRARIES}
    peiskernel_mt
)
