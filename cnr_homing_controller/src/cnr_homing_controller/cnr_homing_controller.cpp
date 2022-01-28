#include <cnr_homing_controller/cnr_homing_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::HomingController,  controller_interface::ControllerBase)


namespace  cnr
{
namespace control
{

//!
bool HomingController::doInit()
{
  CNR_TRACE_START(this->logger());
//  if(this->nAx() > 1)
//    CNR_RETURN_FALSE(this->logger(),
//     "Homing controller is designed for just 1 joint, while "+std::to_string(this->nAx())+" joints configured. Abort.");
  CNR_INFO(this->logger(), "Init homing of joint " << cnr::control::to_string(this->chain().getActiveJointsName()));
  this->setPriority(this->Q_PRIORITY);
  CNR_RETURN_TRUE(this->logger());
}

//!
bool HomingController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  CNR_INFO(this->logger(), "Start homing of joint " << cnr::control::to_string(this->chain().getActiveJointsName()));
  CNR_RETURN_TRUE(this->logger());
}

//!
bool HomingController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  CNR_INFO(this->logger(), "Stop homing of joint " << cnr::control::to_string(this->chain().getActiveJointsName()));
  CNR_RETURN_TRUE(this->logger());
}

//!
bool HomingController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

}
}
