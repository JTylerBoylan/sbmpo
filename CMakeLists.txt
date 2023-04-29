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
    src/sbmpo.cpp
)

# Set include directories for library target
target_include_directories(sbmpo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)