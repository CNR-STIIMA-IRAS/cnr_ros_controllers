#include <ros/ros.h>
#include <ros/time.h>
#include <cnr_controller_interface/internal/utils.h>
#include <cnr_open_loop_position_controller/cnr_open_loop_position_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::OpenLoopPositionController, controller_interface::ControllerBase);


namespace cnr
{
namespace control
{


bool OpenLoopPositionController::doInit()
{

  CNR_TRACE_START(*m_logger);
  if (m_joint_names.size() > 1)
  {
    CNR_RETURN_FALSE(*m_logger,
                     "Error, the number of joints contrlled is more than 1. Check the configuration.");
  }

  for (unsigned idx = 0; idx < m_hw->getNames().size(); idx++)
  {
    if (!m_hw->getNames().at(idx).compare(m_joint_names.front()))
    {
      m_jh = m_hw->getHandle(m_joint_names.front());
    }
  }

  if (!getControllerNh().getParam("setpoint_topic_name", setpoint_topic_name))
  {
    CNR_RETURN_FALSE(*m_logger, getControllerNamespace() + "/'setpoint_topic_name' does not exist");
  }
  add_subscriber(SP_TOPIC_ID, setpoint_topic_name, 1, &OpenLoopPositionController::callback, this);

  CNR_DEBUG(*m_logger, "Controller ' " + getControllerNamespace() + "' controls the following joint: " + cnr_controller_interface::to_string(m_joint_names));
  CNR_DEBUG(*m_logger, "Controller ' " + getControllerNamespace() + "' get the setpoint from the topic: '" + setpoint_topic_name + "'");
  CNR_RETURN_TRUE(*m_logger);
}

bool OpenLoopPositionController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_configured = false;
  m_pos_cmd = m_jh.getPosition();
  CNR_RETURN_TRUE(*m_logger);
}

bool OpenLoopPositionController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  CNR_RETURN_TRUE(*m_logger);
}

bool OpenLoopPositionController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 10.0);
  try
  {
    m_jh.setCommand(m_pos_cmd);
  }
  catch (...)
  {
    CNR_WARN(*m_logger, "something wrong: Controller '" + getControllerNamespace() + "'");
    m_jh.setCommand(m_jh.getPosition());
  }
  CNR_RETURN_TRUE_THROTTLE(*m_logger, 10.0);
}

bool OpenLoopPositionController::extractJoint(const sensor_msgs::JointState msg, const std::string name, double& pos)
{
  for (unsigned int iJoint = 0; iJoint < msg.name.size(); iJoint++)
  {
    if (!msg.name.at(iJoint).compare(name))
    {
      if (msg.position.size() > (iJoint))
        pos = msg.position.at(iJoint);
      else
        return false;

      return true;
    }
  }
  return false;
}

void OpenLoopPositionController::callback(const sensor_msgs::JointStateConstPtr msg)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0);
  tick(SP_TOPIC_ID);
  if (extractJoint(*msg, m_joint_names.front(), m_pos_cmd))
  {
    m_configured = true;
  }
  else
  {
    CNR_FATAL(*m_logger, getControllerNamespace() + " target message dimension is wrong.");
  }
  CNR_RETURN_OK_THROTTLE(*m_logger, void(), 5.0);
}


}
}
