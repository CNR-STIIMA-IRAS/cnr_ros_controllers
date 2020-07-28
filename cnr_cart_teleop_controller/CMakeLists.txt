cmake_minimum_required(VERSION 2.8.3)
project(cnr_cart_teleop_controller)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  urdf
  cnr_controller_interface
  controller_manager
  eigen_state_space_systems
  hardware_interface
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  subscription_notifier
  cnr_hardware_interface
  rosdyn_core
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cnr_joint_teleop_controller
 CATKIN_DEPENDS urdf cnr_controller_interface controller_manager eigen_state_space_systems hardware_interface pluginlib roscpp sensor_msgs std_msgs subscription_notifier cnr_hardware_interface rosdyn_core
 DEPENDS
)

include_directories(
  src
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cnr_cart_teleop_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



