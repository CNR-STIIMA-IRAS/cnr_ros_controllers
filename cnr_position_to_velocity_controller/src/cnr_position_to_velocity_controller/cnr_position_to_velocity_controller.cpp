#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::PositionToVelocityController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::PositionToVelocityControllerFfw, controller_interface::ControllerBase)

namespace cnr
{
namespace control
{


bool PositionToVelocityController::doInit()
{
  CNR_TRACE_START(*m_logger);
  if(nAx()>1)
  {
    CNR_RETURN_FALSE(*m_logger, "The controller is designed to control only one joint.");
  }
  if (!ctrl.init(getRootNh(), getControllerNh()))
  {
    CNR_RETURN_FALSE(*m_logger, "Math ctrl of the PositionToVelocityController failed in initialization.");
  }

  this->setPriority(QD_PRIORITY);
  CNR_RETURN_TRUE(*m_logger);
}

bool PositionToVelocityController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  ctrl.starting(time, q(0), qd(0));
  CNR_RETURN_TRUE(*m_logger);
}

bool PositionToVelocityController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  ctrl.stopping(time);
  setCommandVelocity(0,0);
  CNR_RETURN_TRUE(*m_logger);
}

bool PositionToVelocityController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  try
  {
    ctrl.update(time, period, q(0), qd(0));
    setCommandVelocity(ctrl.getVelCmd(), 0);
  }
  catch (...)
  {
    setCommandVelocity(0,0);
    CNR_RETURN_FALSE_THROTTLE(*m_logger, 2.0, "something wrong: Controller '" + getControllerNamespace() + "'");
  }
  CNR_RETURN_TRUE(*m_logger);
}






bool PositionToVelocityControllerFfw::doInit()
{
  CNR_TRACE_START(*m_logger);
  if(nAx()>1)
  {
    CNR_RETURN_FALSE(*m_logger, "The controller is designed to control only one joint.");
  }
  if (!ctrl.init(getRootNh(), getControllerNh()))
  {
    CNR_RETURN_FALSE(*m_logger, "Math ctrl of the PositionToVelocityControllerFfw failed in initialization.");
    return false;
  }
  CNR_RETURN_TRUE(*m_logger);
}

bool PositionToVelocityControllerFfw::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  ctrl.starting(time, q(0), qd(0));
  CNR_RETURN_TRUE(*m_logger);
}

bool PositionToVelocityControllerFfw::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  ctrl.stopping(time);
  setCommandVelocity(0,0);
  setCommandEffort(0,0);
  CNR_RETURN_TRUE(*m_logger);
}

bool PositionToVelocityControllerFfw::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  try
  {
    ctrl.update(time, period, q(0), qd(0));
    setCommandPosition(ctrl.getPosCmd(),0);
    setCommandVelocity(ctrl.getVelCmd(),0);
    setCommandEffort(ctrl.getEffCmd(),0);
  }
  catch (...)
  {
    setCommandVelocity(0,0);
    setCommandEffort(0,0);
    CNR_RETURN_FALSE_THROTTLE(*m_logger, 2.0, "something wrong: Controller '" + getControllerNamespace() + "'");
  }
  CNR_RETURN_TRUE(*m_logger);
}


}  // namespace control
}  // namespace cnr
