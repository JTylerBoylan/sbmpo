cmake_minimum_required(VERSION 3.5)

# Define project and its version
project(sbmpo_models VERSION 1.0)

# Set C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)

# Find sbmpo library
find_package(sbmpo REQUIRED)

include_directories(include ${sbmpo_INCLUDE_DIRS})

# Grid2D Model Executable
add_executable(grid_2d_benchmark 
  src/grid_2d_benchmark.cpp
)
target_include_directories(grid_2d_benchmark PUBLIC include ${sbmpo_INCLUDE_DIRS})
target_link_libraries(grid_2d_benchmark PUBLIC sbmpo stdc++ stdc++fs m)

# Simple Steering Model Exectuable
add_executable(simple_steering_benchmark
  src/simple_steering_benchmark.cpp
)
target_include_directories(simple_steering_benchmark PUBLIC include ${sbmpo_INCLUDE_DIRS})
target_link_libraries(simple_steering_benchmark PUBLIC sbmpo stdc++ stdc++fs m)
