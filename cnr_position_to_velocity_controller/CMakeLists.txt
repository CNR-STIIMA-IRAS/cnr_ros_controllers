cmake_minimum_required(VERSION 3.1)

project(cnr_position_to_velocity_controller)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  cnr_controller_interface
  cnr_hardware_interface
  eigen_state_space_systems
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  subscription_notifier
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_position_to_velocity_controller
  CATKIN_DEPENDS cnr_controller_interface controller_manager cnr_hardware_interface eigen_state_space_systems 
                 subscription_notifier pluginlib roscpp sensor_msgs std_msgs  urdf
  DEPENDS
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cnr_position_to_velocity_math.cpp
  src/${PROJECT_NAME}/cnr_position_to_velocity_controller.cpp
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
