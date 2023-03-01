cmake_minimum_required(VERSION 3.5)

# Define project and its version
project(sbmpo_benchmarking VERSION 1.0)

include_directories(include)

# Grid2D Model Executable
add_executable(grid_2d_benchmark 
  src/grid_2d_benchmark.cpp
)
target_include_directories(grid_2d_benchmark PUBLIC include)
target_link_libraries(grid_2d_benchmark PUBLIC sbmpo sbmpo_models stdc++ stdc++fs m)

# Simple Steering Model Exectuable
add_executable(simple_steering_benchmark
  src/simple_robot_benchmark.cpp
)
target_include_directories(simple_steering_benchmark PUBLIC include)
target_link_libraries(simple_steering_benchmark PUBLIC sbmpo sbmpo_models stdc++ stdc++fs m)