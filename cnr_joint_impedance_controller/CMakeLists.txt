cmake_minimum_required(VERSION 3.1)
project(cnr_joint_impedance_controller)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)



add_definitions(-DFOURBYTHREE_MSGS)
find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  controller_interface
  controller_manager
  hardware_interface
  rosdyn_core
  roscpp
  sensor_msgs
  subscription_notifier
  cnr_hardware_interface
  name_sorting
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cnr_controller_interface controller_interface controller_manager hardware_interface cnr_hardware_interface rosdyn_core roscpp sensor_msgs subscription_notifier name_sorting
  DEPENDS 
)



include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cnr_joint_impedance_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



