cmake_minimum_required(VERSION 3.1)
project(robot_state_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


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

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/robot_state_controller.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)
