cmake_minimum_required(VERSION 3.15)

# Set C++ standard library
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake and load its package
find_package(ament_cmake REQUIRED)

# Define project and its version
project(sbmpo VERSION 1.0)

# Use Release mode
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Specify the directory containing header files for your library
include_directories(include)

# Create the library target and specify source files
add_library(sbmpo SHARED
  src/grids/ImplicitGrid.cpp
  src/queues/PriorityQueue.cpp
  src/algorithms/Astar.cpp
  src/models/UnicycleSteeringModel.cpp
  src/models/AckermannSteeringModel.cpp
  src/models/DoubleIntegratorModel.cpp
)

# Set include directories for library target
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Export the package so it can be used by other packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)

# Install the created library and its headers
install(
  TARGETS sbmpo
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Generate ament package environment files
ament_package()
