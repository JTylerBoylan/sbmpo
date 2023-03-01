cmake_minimum_required(VERSION 3.5)

if(NOT CMAKE_CXX_COMPILER)
    set(CMAKE_CXX_COMPILER "/usr/bin/gcc")
endif()

project(SBMPO)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
    add_compile_options(-Wall)
endif()

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add subdirectories for each project
add_subdirectory(sbmpo)
add_subdirectory(sbmpo_models)
add_subdirectory(sbmpo_benchmarking)