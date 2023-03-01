cmake_minimum_required(VERSION 3.5)

# Define project and its version
project(sbmpo_models VERSION 1.0)

# Use C++11
set(CMAKE_CXX_STANDARD 11)

# Specify the directory containing header files for your library
include_directories(include)

# Get all header files in the include/sbmpo_models directory
file(GLOB SBMPO_MODELS_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/sbmpo_models/*.hpp")

# Create a custom target for sbmpo_models that only includes the header files
add_library(sbmpo_models INTERFACE)
target_include_directories(sbmpo_models INTERFACE .)

# Add link to sbmpo
target_link_libraries(sbmpo_models INTERFACE sbmpo)

# Set include directories for library target
target_include_directories(sbmpo_models INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set library versioning
set_target_properties(sbmpo_models PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)

