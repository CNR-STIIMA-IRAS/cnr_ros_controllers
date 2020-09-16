cmake_minimum_required(VERSION 2.8.3)
project(cnr_velocity_to_torque_controller)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  controller_interface
  controller_manager
  hardware_interface
  subscription_notifier
  pluginlib
  roscpp
  std_msgs
  
  sensor_msgs
  eigen_state_space_systems
  
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_velocity_to_torque_controller
  CATKIN_DEPENDS cnr_controller_interface controller_interface controller_manager hardware_interface
  eigen_state_space_systems  subscription_notifier pluginlib roscpp sensor_msgs std_msgs
  DEPENDS 
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cnr_velocity_to_torque_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".git" EXCLUDE
 )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})
