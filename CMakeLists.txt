cmake_minimum_required(VERSION 3.0.2)
project(sbmpo)

## Compile as C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optional: use -Wall -Wextra -Wpedantic
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find catkin macros and libraries
find_package(catkin REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# Create the library target and specify source files
add_library(sbmpo
  src/grids/ImplicitGrid.cpp
  src/queues/PriorityQueue.cpp
  src/algorithms/Astar.cpp
  src/models/UnicycleSteeringModel.cpp
  src/models/AckermannSteeringModel.cpp
  src/models/DoubleIntegratorModel.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)