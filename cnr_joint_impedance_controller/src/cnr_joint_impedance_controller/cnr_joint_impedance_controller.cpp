#include <name_sorting/name_sorting.h>
#include <cnr_joint_impedance_controller/cnr_joint_impedance_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointImpedanceController , controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::JointImpedanceController1, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::JointImpedanceController3, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::JointImpedanceController6, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::JointImpedanceController7, controller_interface::ControllerBase)
