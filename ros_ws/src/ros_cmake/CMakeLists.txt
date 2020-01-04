cmake_minimum_required(VERSION 2.8.3)
project(ros_cmake)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
add_definitions(-Wall)
add_definitions(-O0)
add_definitions(-g)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)



## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5Widgets)   # Qt Environment

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  include
)


qt5_wrap_ui(UIC widget.ui)

add_executable(${PROJECT_NAME}  src/main.cpp src/widget.cpp src/ui_widget.h src/widget.h src/widget.ui)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} Qt5::Widgets)


