cmake_minimum_required(VERSION 3.5)

# Define project and its version
project(sbmpo_ros VERSION 1.0)

# Include directories
include_directories(include)

# Create ament package
find_package(ament_cmake REQUIRED)

# Create executable
#add_executable(sbmpo_ros_node src/sbmpo_ros_node.cpp)
#target_link_libraries(sbmpo_ros_node PUBLIC sbmpo)

# Install executable
#install(TARGETS sbmpo_ros_node
#        DESTINATION lib/${PROJECT_NAME}
#)

# Install include directory
#install(DIRECTORY include/
#        DESTINATION include/
#)

# Export targets
#ament_export_targets(sbmpo)
#ament_export_include_directories(include)
#ament_export_libraries(sbmpo)

# Create ament package
ament_package()
