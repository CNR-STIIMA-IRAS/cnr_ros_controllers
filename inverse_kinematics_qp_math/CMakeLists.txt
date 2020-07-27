cmake_minimum_required(VERSION 2.8.3)
project(inverse_kinematics_qp_math)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  eigen_matrix_utils
  roscpp
  sensor_msgs
  rosdyn_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS eigen_matrix_utils roscpp sensor_msgs rosdyn_core
)

include_directories(
  include  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/inverse_kinematics_qp_math.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} )

add_executable(test_inverse_kinematics_qp_math_node src/test_inverse_kinematics_qp_math_node.cpp)
add_dependencies(test_inverse_kinematics_qp_math_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_inverse_kinematics_qp_math_node  ${PROJECT_NAME} ${catkin_LIBRARIES} )

FILE(GLOB_RECURSE YamlFiles "config/*.yaml")
add_custom_target(yaml SOURCES ${YamlFiles})

FILE(GLOB_RECURSE LaunchFiles "launch/*.launch")
add_custom_target(launch SOURCES ${LaunchFiles})
