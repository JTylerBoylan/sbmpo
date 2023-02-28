cmake_minimum_required(VERSION 3.5)

# Define project and its version
project(sbmpo VERSION 1.0 DESCRIPTION "SBMPO Library")

include(GNUInstallDirs)

add_library(${PROJECT_NAME}
    src/sbmpo.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)

install(TARGETS ${PROJECT_NAME} EXPORT sbmpo
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/sbmpo/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

# Install package configuration files
install(EXPORT sbmpo
    DESTINATION lib/cmake/sbmpo
    FILE sbmpoConfig.cmake)

export(TARGETS ${PROJECT_NAME} FILE sbmpoConfig.cmake)
