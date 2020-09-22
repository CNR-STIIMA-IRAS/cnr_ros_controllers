#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::PositionToVelocityController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::PositionToVelocityControllerFfw, controller_interface::ControllerBase)

namespace cnr
{
namespace control
{


/**
 * @brief PositionToVelocityControllerFfw::doInit
 * @return
 */
bool PositionToVelocityControllerFfw::doInit()
{
  CNR_TRACE_START(this->logger());
  if(PositionToVelocityControllerBase::doInit())
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

bool PositionToVelocityControllerFfw::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(PositionToVelocityControllerBase::doStarting(time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

bool PositionToVelocityControllerFfw::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(PositionToVelocityControllerBase::doStopping(time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  this->setCommandVelocity(0,0);
  this->setCommandEffort(0,0);
  CNR_RETURN_TRUE(this->logger());
}

bool PositionToVelocityControllerFfw::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    if(PositionToVelocityControllerBase::doUpdate(time, period))
    {
      CNR_RETURN_FALSE(this->logger());
    }
    this->setCommandPosition(ctrl.getPosCmd(),0);
    this->setCommandVelocity(ctrl.getVelCmd(),0);
    this->setCommandEffort(ctrl.getEffCmd(),0);
  }
  catch (...)
  {
    this->setCommandVelocity(0,0);
    this->setCommandEffort(0,0);
    CNR_RETURN_FALSE_THROTTLE(this->logger(), 2.0, "something wrong: Controller '" + getControllerNamespace() + "'");
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}






}  // namespace control
}  // namespace cnr
