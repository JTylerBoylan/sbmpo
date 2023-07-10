cmake_minimum_required(VERSION 3.15)

# Set C++ standard library
set(CMAKE_CXX_STANDARD 20)

# Define project and its version
project(sbmpo VERSION 1.0)

# Use Release mode
set(CMAKE_BUILD_TYPE Release)

# Specify the directory containing header files for your library
include_directories(include)

# Create the library target and specify source files
add_library(sbmpo SHARED)
target_sources(sbmpo PRIVATE 
    src/Astar.cpp
)

# Set include directories for library target
target_include_directories(sbmpo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Unicycle Planner Test Executable
add_executable(unicycle_planner_test
  src/examples/unicycle_planner_test.cpp
)
target_link_libraries(unicycle_planner_test PUBLIC sbmpo stdc++ stdc++fs m)

# Unicycle Steering Benchmark Example Executable
add_executable(unicycle_steering_benchmark_example
  src/examples/unicycle_steering_benchmark.cpp
)
target_link_libraries(unicycle_steering_benchmark_example PUBLIC sbmpo stdc++ stdc++fs m)

# Ackermann Steering Benchmark Example Executable
add_executable(ackermann_steering_benchmark_example
  src/examples/ackermann_steering_benchmark.cpp
)
target_link_libraries(ackermann_steering_benchmark_example PUBLIC sbmpo stdc++ stdc++fs m)

# Unicycle Planner Benchmark Example Executable
add_executable(double_integrator_benchmark_example
  src/examples/double_integrator_benchmark.cpp
)
target_link_libraries(double_integrator_benchmark_example PUBLIC sbmpo stdc++ stdc++fs m)