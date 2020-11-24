cmake_minimum_required(VERSION 3.1)
project(cnr_joint_impedance_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_impedance_regulator
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

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cnr_controller_interface controller_interface controller_manager hardware_interface cnr_hardware_interface rosdyn_core roscpp sensor_msgs subscription_notifier name_sorting
  DEPENDS 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_joint_impedance_controller.cpp )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall $<$<CONFIG:RELEASE>:-Ofast>)
#target_compile_definitions(${PROJECT_NAME} PUBLIC -DEIGEN_DONT_VECTORIZE=1)



