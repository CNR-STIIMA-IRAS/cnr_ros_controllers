#include <cnr_homing_controller/cnr_homing_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::HomingController,  controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::HomingController1, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::HomingController3, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::HomingController6, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::HomingController7, controller_interface::ControllerBase)


namespace  cnr
{
namespace control
{

}
}
