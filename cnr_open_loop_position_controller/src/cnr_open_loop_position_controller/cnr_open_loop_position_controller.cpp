#include <ros/ros.h>
#include <ros/time.h>
#include <cnr_controller_interface/internal/utils.h>
#include <cnr_open_loop_position_controller/cnr_open_loop_position_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::OpenLoopPositionController, controller_interface::ControllerBase)



namespace cnr
{
namespace control
{


bool OpenLoopPositionController::doInit()
{
  CNR_TRACE_START(*m_logger);
  if(!getControllerNh().getParam("setpoint_topic_name", m_setpoint_topic_name))
  {
    CNR_RETURN_FALSE(*m_logger, "The param '" + getControllerNamespace() + "/setpoint_topic_name' does not exist");
  }
  add_subscriber<sensor_msgs::JointState>(m_setpoint_topic_name, 1,
        boost::bind(&OpenLoopPositionController::callback, this, _1) );

  this->setPriority(Q_PRIORITY);

  CNR_DEBUG(*m_logger, "Controller ' " + getControllerNamespace() + "' controls the following joint: "
                     + cnr_controller_interface::to_string(jointNames()));
  CNR_DEBUG(*m_logger, "Controller ' " + getControllerNamespace() + "' get the setpoint from the topic: '"
                     + m_setpoint_topic_name + "'");
  CNR_RETURN_TRUE(*m_logger);
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
  m_configured = false;
  CNR_RETURN_TRUE(*m_logger);
}

bool OpenLoopPositionController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(*m_logger);

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(*m_logger);
}

bool OpenLoopPositionController::extractJoint(const sensor_msgs::JointState& msg)
{
  size_t cnt = 0;
  Eigen::VectorXd target = getCommandPosition();
  for (size_t iJoint=0; iJoint < msg.name.size(); iJoint++)
  {
    for (size_t iAx=0; iAx < jointNames().size(); iAx++)
    {
      if(msg.name.at(iJoint) == jointNames().at(iAx))
      {
        if(msg.position.size() > (iJoint))
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
  if( ok )
  {
    setCommandPosition(target);
  }
  return ok;
}

void OpenLoopPositionController::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 1.0);
  if(extractJoint(*msg))
  {
    m_configured = true;
  }
  else
  {
    CNR_FATAL(*m_logger, getControllerNamespace() + " command message dimension is wrong.");
  }
  CNR_RETURN_OK_THROTTLE(*m_logger, void(), 1.0);
}


}
}
