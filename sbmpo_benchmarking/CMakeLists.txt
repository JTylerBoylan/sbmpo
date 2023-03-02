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

# Simple Steering Model Executable
add_executable(simple_robot_benchmark
  src/simple_robot_benchmark.cpp
)
target_include_directories(simple_robot_benchmark PUBLIC include)
target_link_libraries(simple_robot_benchmark PUBLIC sbmpo sbmpo_models stdc++ stdc++fs m)

# Double Integrator Model Executable
add_executable(double_integrator_benchmark
  src/double_integrator_benchmark.cpp
)
target_include_directories(double_integrator_benchmark PUBLIC include)
target_link_libraries(double_integrator_benchmark PUBLIC sbmpo sbmpo_models stdc++ stdc++fs m)