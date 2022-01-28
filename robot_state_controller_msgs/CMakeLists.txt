cmake_minimum_required(VERSION 3.0.2)
project(robot_state_controller_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
 FILES
 PoseTwistArray.msg
 PoseTwist.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_state_controller_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
