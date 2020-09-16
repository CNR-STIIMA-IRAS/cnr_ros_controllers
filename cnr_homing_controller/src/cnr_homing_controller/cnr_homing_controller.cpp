#include <cnr_homing_controller/cnr_homing_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::HomingController, controller_interface::ControllerBase)


namespace  cnr
{
namespace control
{


bool HomingController::doInit()
{
  CNR_TRACE_START(*m_logger);
  if(nAx() > 1)
    CNR_RETURN_FALSE(*m_logger,
        "Homing controller is designed for just 1 joint, while "+std::to_string(nAx())+" joints configured. Abort.");
  CNR_INFO(*m_logger, "Init homing of joint " << jointName(0));
  this->setPriority(Q_PRIORITY);
  CNR_RETURN_TRUE(*m_logger)
}


bool HomingController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  CNR_INFO(*m_logger, "Start homing of joint " << jointName(0));
  CNR_RETURN_TRUE(*m_logger)
}

bool HomingController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  CNR_INFO(*m_logger, "Stop homing of joint " << jointName(0));
  CNR_RETURN_TRUE(*m_logger)
}

bool HomingController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_RETURN_TRUE(*m_logger)
}
    
}
}
