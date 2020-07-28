cmake_minimum_required(VERSION 2.8.3)
project(inverse_kinematics_qp_controller)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  eigen_matrix_utils
  geometry_msgs
  inverse_kinematics_qp_math
  roscpp
  rosdyn_core
  sensor_msgs
  subscription_notifier
  cnr_hardware_interface
  cnr_controller_interface
  name_sorting
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cnr_controller_interface eigen_conversions eigen_matrix_utils geometry_msgs inverse_kinematics_qp_math roscpp rosdyn_core sensor_msgs subscription_notifier cnr_hardware_interface name_sorting
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/inverse_kinematics_qp_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} )

