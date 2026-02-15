cmake_minimum_required(VERSION 3.5)

project(QtRos2Example VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Default to Release build if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

find_package(Qt5 COMPONENTS Core Quick REQUIRED)
# find_package(rclcpp REQUIRED)

set(PROJECT_SOURCES
    main.cpp
    MyNode.cpp
    MyNode.h
    qml.qrc
)

qt5_add_resources(RESOURCES qml.qrc)

add_executable(QtRos2Example ${PROJECT_SOURCES} ${RESOURCES})

target_link_libraries(QtRos2Example
    Qt5::Core
    Qt5::Quick
    # rclcpp::rclcpp
)

install(TARGETS QtRos2Example
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

