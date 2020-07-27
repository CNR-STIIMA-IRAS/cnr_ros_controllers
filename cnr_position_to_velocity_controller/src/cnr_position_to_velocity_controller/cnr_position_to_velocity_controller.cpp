#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::PositionToVelocityController, controller_interface::ControllerBase);
PLUGINLIB_EXPORT_CLASS(cnr::control::PositionToVelocityControllerFfw, controller_interface::ControllerBase);

namespace cnr
{
namespace control
{


bool PositionToVelocityController::doInit()
{
  if (!ctrl.init(getRootNh(), getControllerNh()))
  {
    CNR_FATAL(*m_logger, "DoInit of the math ctrl of the PositionToVelocityController failed in initialization.");
    return false;
  }

  bool flag = false;
  for (unsigned idx = 0; idx < m_hw->getNames().size(); idx++)
  {
    if (!m_hw->getNames().at(idx).compare(ctrl.getJointName()))
    {
      m_jh = m_hw->getHandle(ctrl.getJointName());
      flag = true;
      break;
    }
  }
  if (!flag)
  {
    CNR_FATAL(*m_logger, "There is an error in the names of the controlled joints, they mismatch with the handles.");
    return false;
  }
  return true;
}

bool PositionToVelocityController::doStarting(const ros::Time& time)
{
  // CHECK IF JOINT NAME IS PRESENT
  double fb_pos = m_jh.getPosition();
  double fb_vel = m_jh.getVelocity();
  ctrl.starting(time, fb_pos, fb_vel);
  return true;
}

bool PositionToVelocityController::doStopping(const ros::Time& time)
{
  ctrl.stopping(time);
  m_jh.setCommand(0);
  return true;
}

bool PositionToVelocityController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  double fb_pos = m_jh.getPosition();
  double fb_vel = m_jh.getVelocity();

  try
  {
    ctrl.update(time, period, fb_pos, fb_vel);
    m_jh.setCommand(ctrl.getVelCmd());
  }
  catch (...)
  {
    CNR_WARN_THROTTLE(*m_logger, 2.0, "something wrong: Controller '" + getControllerNamespace() + "'");
    m_jh.setCommand(0);
    return false;
  }
  return true;
}






bool PositionToVelocityControllerFfw::doInit()
{
  if (!ctrl.init(getRootNh(), getControllerNh()))
  {
    CNR_ERROR(*m_logger, "unable to initialize controller");
    return false;
  }
  bool flag = false;
  for (unsigned idx = 0; idx < m_hw->getNames().size(); idx++)
  {
    if (!m_hw->getNames().at(idx).compare(ctrl.getJointName()))
    {
      m_jh = m_hw->getHandle(ctrl.getJointName());
      flag = true;
      break;
    }
  }
  if (!flag)
  {
    CNR_FATAL(*m_logger, "There is an error in the names of the controlled joints, they mismatch with the handles.");
    return false;
  }
  return true;
}

bool PositionToVelocityControllerFfw::doStarting(const ros::Time& time)
{
  // CHECK IF JOINT NAME IS PRESENT
  double fb_pos = m_jh.getPosition();
  double fb_vel = m_jh.getVelocity();
  ctrl.starting(time, fb_pos, fb_vel);
  return true;
}

bool PositionToVelocityControllerFfw::doStopping(const ros::Time& time)
{
  ctrl.stopping(time);
  m_jh.setCommandVelocity(0);
  m_jh.setCommandEffort(0);
  return true;
}

bool PositionToVelocityControllerFfw::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  double fb_pos = m_jh.getPosition();
  double fb_vel = m_jh.getVelocity();

  try
  {
    ctrl.update(time, period, fb_pos, fb_vel);
    m_jh.setCommandPosition(ctrl.getPosCmd());
    m_jh.setCommandVelocity(ctrl.getVelCmd());
    m_jh.setCommandEffort(ctrl.getEffCmd());
  }
  catch (...)
  {
    CNR_WARN_THROTTLE(*m_logger, 2.0, "something wrong: Controller '" + getControllerNamespace() + "'");
    m_jh.setCommandVelocity(0);
    m_jh.setCommandEffort(0);
    return false;
  }
  return true;
}

}
}
