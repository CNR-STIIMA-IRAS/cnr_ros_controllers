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
  for (size_t iHw = 0; iHw < m_hw->getNames().size(); iHw++)
  {
    for (size_t iAx=0; iAx < nAx(); iAx++)
    {
      if ( m_hw->getNames().at(iHw) == jointNames().at(iAx) )
      {
        m_hw->getHandle(jointNames().at(iAx) );
      }
    }
  }

  if (!getControllerNh().getParam("setpoint_topic_name", setpoint_topic_name))
  {
    CNR_RETURN_FALSE(*m_logger, getControllerNamespace() + "/'setpoint_topic_name' does not exist");
  }
  add_subscriber(SP_TOPIC_ID, setpoint_topic_name, 1, &OpenLoopPositionController::callback, this);

  CNR_DEBUG(*m_logger, "Controller ' " + getControllerNamespace() + "' controls the following joint: " + cnr_controller_interface::to_string(jointNames()));
  CNR_DEBUG(*m_logger, "Controller ' " + getControllerNamespace() + "' get the setpoint from the topic: '" + setpoint_topic_name + "'");
  CNR_RETURN_TRUE(*m_logger);
}

void OpenLoopPositionController::updateJointKinematicStatus  ( Eigen::VectorXd& q,
                                                               Eigen::VectorXd& qd,
                                                               Eigen::VectorXd& qdd,
                                                               Eigen::VectorXd& effort )
{
  for (size_t iAx = 0; iAx <nAx(); iAx++)
  {
    q(iAx) = m_hw->getHandle(jointNames().at(iAx) ).getPosition();
    qd(iAx) = m_hw->getHandle(jointNames().at(iAx) ).getVelocity();
    qdd(iAx) = 0.0;
    effort(iAx) = m_hw->getHandle(jointNames().at(iAx) ).getEffort();
  }
}

bool OpenLoopPositionController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_configured = false;
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

  CNR_RETURN_TRUE_THROTTLE(*m_logger, 10.0);
}

bool OpenLoopPositionController::extractJoint(const sensor_msgs::JointState& msg)
{
  size_t cnt = 0;
  Eigen::VectorXd target = getPositionCommand();
  for (size_t iJoint=0; iJoint < msg.name.size(); iJoint++)
  {
    for (size_t iAx=0; iAx < jointNames().size(); iAx++)
    {
      if (msg.name.at(iJoint) == jointNames().at(iAx))
      {
        if (msg.position.size() > (iJoint))
        {
          target(iAx) = msg.position.at(iJoint);
          cnt++;
        }
        else
        {
          return false;
        }
      }
    }
  }

  bool ok = (cnt == nAx());
  if ( ok )
  {
    setPositionCommand(target);
  }
  return ok;
}

void OpenLoopPositionController::callback(const sensor_msgs::JointStateConstPtr msg)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0);
  tick(SP_TOPIC_ID);
  if (extractJoint(*msg))
  {
    m_configured = true;
  }
  else
  {
    CNR_FATAL(*m_logger, getControllerNamespace() + " command message dimension is wrong.");
  }
  CNR_RETURN_OK_THROTTLE(*m_logger, void(), 5.0);
}


}
}
