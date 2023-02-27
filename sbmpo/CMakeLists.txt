cmake_minimum_required(VERSION 3.5)

# Define project and its version
project(sbmpo VERSION 1.0)

# Use C++11
set(CMAKE_CXX_STANDARD 11)

# Specify the directory containing header files for your library
include_directories(include)

# Create the library target and specify source files
add_library(sbmpo SHARED)
target_sources(sbmpo PRIVATE src/sbmpo.cpp)

# Set include directories for library target
target_include_directories(sbmpo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set library versioning
set_target_properties(sbmpo PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)

# Link any necessary dependencies (example: Boost)
# target_link_libraries(sbmpo PRIVATE Boost::system)
