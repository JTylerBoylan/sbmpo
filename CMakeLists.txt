cmake_minimum_required(VERSION 3.15)

# Define project and its version
project(sbmpo VERSION 1.0)

# Set C++ standard library
set(CMAKE_CXX_STANDARD 20)

# Set the installation prefix (this can be overridden by the user)
set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Installation Directory")

####################
## SHARED LIBRARY ##
####################

# Specify the directory containing header files for your library
include_directories(include)

# Create the library target and specify source files
add_library(sbmpo SHARED)
target_sources(sbmpo PRIVATE 
  src/Astar.cpp
)

# Link the library target to the standard C++ library
target_link_libraries(sbmpo PUBLIC stdc++ stdc++fs m)

# Set include directories for library target
target_include_directories(sbmpo PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

#################
## EXECUTABLES ##
#################

# Unicycle Planner Test Executable
add_executable(unicycle_planner_test
  src/examples/unicycle_planner_test.cpp
)
target_link_libraries(unicycle_planner_test PUBLIC sbmpo)

# Unicycle Steering Benchmark Example Executable
add_executable(unicycle_steering_benchmark_example
  src/examples/unicycle_steering_benchmark.cpp
)
target_link_libraries(unicycle_steering_benchmark_example PUBLIC sbmpo)

# Ackermann Steering Benchmark Example Executable
add_executable(ackermann_steering_benchmark_example
  src/examples/ackermann_steering_benchmark.cpp
)
target_link_libraries(ackermann_steering_benchmark_example PUBLIC sbmpo)

# Unicycle Planner Benchmark Example Executable
add_executable(double_integrator_benchmark_example
  src/examples/double_integrator_benchmark.cpp
)
target_link_libraries(double_integrator_benchmark_example PUBLIC sbmpo)

#####################
## LIBRARY INSTALL ##
#####################

# Install the library to the lib directory and associate it with the export set
install(TARGETS sbmpo
  EXPORT sbmpoTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Install header files to the include directory
install(DIRECTORY include/ DESTINATION include)

######################
## EXPORTING TARGET ##
######################

# Include the CMake Package Configuration module if needed
include(CMakePackageConfigHelpers)

# Generate a package config file for find_package()
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/sbmpoConfigVersion.cmake"
  VERSION ${sbmpo_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Export the library targets to a file
install(EXPORT sbmpoTargets
        FILE sbmpoTargets.cmake
        NAMESPACE sbmpo::
        DESTINATION lib/cmake/sbmpo)

# Install the version file
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/sbmpoConfigVersion.cmake"
        DESTINATION lib/cmake/sbmpo)