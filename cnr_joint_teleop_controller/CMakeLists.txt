cmake_minimum_required(VERSION 3.1)

project(cnr_joint_teleop_controller)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  urdf
  cnr_controller_interface
  eigen_state_space_systems
  cnr_hardware_interface
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  name_sorting
  joint_limits_interface
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cnr_joint_teleop_controller
 CATKIN_DEPENDS urdf cnr_controller_interface eigen_state_space_systems cnr_hardware_interface pluginlib roscpp sensor_msgs std_msgs name_sorting joint_limits_interface
 DEPENDS
)

include_directories(
src
include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cnr_joint_teleop_controller.cpp
)


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
