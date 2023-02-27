cmake_minimum_required(VERSION 3.5)

# Define project and its version
project(sbmpo_models VERSION 1.0)

# Set C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)

# Enable building outside the source directory
cmake_policy(SET CMP0079 NEW)

# Include directories
include_directories(include)

# Grid2D Model Executable
add_executable(grid_2d_benchmark 
  src/grid_2d_benchmark.cpp
)
target_link_libraries(grid_2d_benchmark PUBLIC sbmpo stdc++ stdc++fs m)

# Simple Steering Model Exectuable
add_executable(simple_steering_benchmark
  src/simple_steering_benchmark.cpp
)
target_link_libraries(simple_steering_benchmark PUBLIC sbmpo stdc++ stdc++fs m)
