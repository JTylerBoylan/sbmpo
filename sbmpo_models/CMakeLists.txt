cmake_minimum_required(VERSION 3.0.2)
project(sbmpo_models)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sbmpo
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sbmpo_models
  CATKIN_DEPENDS roscpp roslib sbmpo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(benchmark src/benchmarking_node.cpp)
target_link_libraries(benchmark ${catkin_LIBRARIES})

