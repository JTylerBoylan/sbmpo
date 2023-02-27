cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_COMPILER "/usr/bin/gcc")

project(SBMPO)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_BUILD_TYPE Release)

# Add subdirectories for each project
add_subdirectory(sbmpo)
add_subdirectory(sbmpo_models)