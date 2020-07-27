cmake_minimum_required(VERSION 2.8.3)
project(simple_touch_controller_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  geometry_msgs
  actionlib_msgs
  std_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )

## Generate actions in the 'action' folder
add_action_files( DIRECTORY 	action
  FILES
  simpleTouch.action
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES eureca_assembly_msgs
   CATKIN_DEPENDS actionlib_msgs geometry_msgs message_generation std_msgs 
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




