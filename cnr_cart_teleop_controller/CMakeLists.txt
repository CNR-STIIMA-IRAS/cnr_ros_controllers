cmake_minimum_required(VERSION 2.8.3)
project(cnr_cart_teleop_controller)
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  urdf
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  controller_interface
  tf
  tf_conversions
  eigen_state_space_systems
  cnr_hardware_interface
  inverse_kinematics_qp_controller
  rosdyn_core
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS  urdf
                 pluginlib
                 roscpp
                 sensor_msgs
                 std_msgs
                 controller_interface
                 tf
                 tf_conversions
                 eigen_state_space_systems
                 cnr_hardware_interface
                 inverse_kinematics_qp_controller
                 rosdyn_core
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



