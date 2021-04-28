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
if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0") 
	target_precompile_headers(${PROJECT_NAME} PUBLIC
    <cnr_controller_interface/cnr_joint_controller_interface.h>
    <hardware_interface/joint_state_interface.h>
    <hardware_interface/joint_command_interface.h>
    <ros/time.h>
    <ros/duration.h>
    <sensor_msgs/JointState.h>
    <geometry_msgs/TwistStamped.h>
    <rosdyn_core/primitives.h>
    <rosdyn_core/urdf_parser.h>
  )
endif()



## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".git" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

