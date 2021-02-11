cmake_minimum_required(VERSION 3.1)

project(cnr_joint_teleop_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  urdf
  cnr_controller_interface
  state_space_controllers
  cnr_hardware_interface
  pluginlib
  sensor_msgs
  std_msgs
  name_sorting
  joint_limits_interface
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cnr_joint_teleop_controller
 CATKIN_DEPENDS urdf cnr_controller_interface state_space_controllers cnr_hardware_interface pluginlib sensor_msgs std_msgs name_sorting joint_limits_interface
 DEPENDS EIGEN3
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_joint_teleop_controller.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new $<$<CONFIG:RELEASE>:-Ofast>)
#target_compile_definitions(${PROJECT_NAME} PUBLIC -DEIGEN_DONT_VECTORIZE=1)
