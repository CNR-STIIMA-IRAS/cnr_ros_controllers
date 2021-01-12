#include <robot_state_controller/robot_state_controller.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE

PLUGINLIB_EXPORT_CLASS(cnr::control::RobotStateController  , controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::RobotStateController1 , controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::RobotStateController3 , controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::RobotStateController6 , controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::RobotStateController7 , controller_interface::ControllerBase)

