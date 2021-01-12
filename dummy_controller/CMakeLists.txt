cmake_minimum_required(VERSION 2.8.3)
project(dummy_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dummy_controller
  CATKIN_DEPENDS cnr_controller_interface controller_interface controller_manager hardware_interface pluginlib roscpp
  DEPENDS 
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/dummy_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
