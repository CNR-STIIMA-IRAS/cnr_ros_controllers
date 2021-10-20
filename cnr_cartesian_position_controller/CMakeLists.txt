cmake_minimum_required(VERSION 3.1)

project(cnr_cartesian_position_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)





find_package(catkin REQUIRED COMPONENTS
  urdf
  cnr_controller_interface
  state_space_filters
  cnr_hardware_interface
  pluginlib
  geometry_msgs
  sensor_msgs
  std_msgs
  name_sorting
  joint_limits_interface
  tf
  tf_conversions
  actionlib
  actionlib_msgs
  eigen_conversions
)

add_action_files( DIRECTORY 	action
  FILES
  RelativeMove.action
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

generate_messages( DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cnr_cartesian_position_controller
 CATKIN_DEPENDS urdf cnr_controller_interface state_space_filters cnr_hardware_interface pluginlib
                  geometry_msgs sensor_msgs std_msgs name_sorting joint_limits_interface tf tf_conversions actionlib_msgs actionlib eigen_conversions
 DEPENDS EIGEN3
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_cartesian_position_controller.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
  target_precompile_headers(${PROJECT_NAME} PUBLIC
    <cmath>
    <Eigen/Core>
    <cnr_logger/cnr_logger.h>
    <ros/time.h>
    <sensor_msgs/JointState.h>
  )
endif()



#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roscpp REQUIRED)
  find_package(rostest REQUIRED)
  find_package(cnr_fake_hardware_interface REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${cnr_fake_hardware_interface_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${cnr_fake_hardware_interface_LIBRARIES})

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()
