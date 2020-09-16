cmake_minimum_required(VERSION 2.8.3)
project(robot_state_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  controller_interface
  hardware_interface
  roscpp
  rosdyn_core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_state_controller
  CATKIN_DEPENDS cnr_controller_interface controller_interface hardware_interface roscpp rosdyn_core
  DEPENDS
  )
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/robot_state_controller.cpp
  )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )