cmake_minimum_required(VERSION 3.1.0)
project(analog_state_publisher)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  controller_interface
  controller_manager
  hardware_interface
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  subscription_notifier
  cnr_hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES analog_state_publisher
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface pluginlib roscpp sensor_msgs std_msgs subscription_notifier cnr_controller_interface cnr_hardware_interface
  DEPENDS
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/analog_state_publisher.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
#target_compile_definitions(${PROJECT_NAME} PUBLIC -DEIGEN_DONT_VECTORIZE=1)

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

