cmake_minimum_required(VERSION 3.1)

project(cnr_cartesian_motion_ctrl)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  controller_interface
  geometry_msgs
  cnr_hardware_interface
  cnr_cart_teleop_controller
  roscpp
  rosdyn_core
  subscription_notifier
  actionlib
  cartesian_motion_msgs
  tf_conversions
  tf
  inverse_kinematics_qp_controller
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS inverse_kinematics_qp_controller cnr_controller_interface controller_interface geometry_msgs cnr_cart_teleop_controller cnr_hardware_interface roscpp rosdyn_core subscription_notifier actionlib cartesian_motion_msgs tf_conversions tf
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

 add_library(${PROJECT_NAME}
   src/cnr_cartesian_motion_ctrl.cpp
 )
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )

 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

 install(FILES
   plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

