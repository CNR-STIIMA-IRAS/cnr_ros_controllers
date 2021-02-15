cmake_minimum_required(VERSION 2.8.3)
project(cnr_velocity_to_torque_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


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
  state_space_controllers
  state_space_ros
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_velocity_to_torque_controller
  CATKIN_DEPENDS cnr_controller_interface controller_interface controller_manager hardware_interface
       state_space_controllers state_space_ros  subscription_notifier pluginlib roscpp sensor_msgs std_msgs
  DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_velocity_to_torque_controller.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math -march=native -mavx -mfma>)

if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0") 
  target_precompile_headers(${PROJECT_NAME} PUBLIC #include <thread>
    <mutex>
    <vector>
    <string>
    <ros/time.h>
    <ros/duration.h>
    <sensor_msgs/JointState.h>

    <subscription_notifier/subscription_notifier.h>
    <state_space_ros/ros_params.h>
    <state_space_controllers/controllers.h>
    <hardware_interface/joint_command_interface.h>
    <cnr_controller_interface/cnr_joint_command_controller_interface.h>>)
endif()

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
