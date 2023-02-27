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

add_executable(grid_2d_benchmark src/grid_2d_benchmark.cpp)
target_link_libraries(grid_2d_benchmark ${catkin_LIBRARIES})

add_executable(simple_steering_benchmark src/simple_steering_benchmark.cpp)
target_link_libraries(simple_steering_benchmark ${catkin_LIBRARIES})

