#include <ros/ros.h>
#include <ros/time.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_controller_interface/utils/utils.h>
#include <cnr_open_loop_position_controller/cnr_open_loop_position_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::OpenLoopPositionController, controller_interface::ControllerBase)

namespace cnr
{
namespace control
{

//!
//! \brief OpenLoopPositionController::doInit
//! \return
//!
inline bool OpenLoopPositionController::doInit()
{
  CNR_TRACE_START(this->m_logger);
  if(!this->getControllerNh().getParam("setpoint_topic_name", m_setpoint_topic_name))
  {
    CNR_RETURN_FALSE(this->m_logger,"The param '"+this->getControllerNamespace()+"/setpoint_topic_name' does not exist");
  }
  bool setpoint_watchdog=true;
  if(!this->getControllerNh().getParam("enable_setpoint_watchdog", setpoint_watchdog))
  {
    CNR_DEBUG(this->m_logger,"The param '"+this->getControllerNamespace()+"/enable_setpoint_watchdog' does not exist, enable watchdog by default");
    setpoint_watchdog=true;
  }
  this->template add_subscriber<sensor_msgs::JointState>(m_setpoint_topic_name, 1,
      boost::bind(&OpenLoopPositionController::callback, this, _1),
      setpoint_watchdog);

  this->setPriority(this->Q_PRIORITY);


  CNR_DEBUG(this->m_logger, "Controller ' "+this->getControllerNamespace()+"' controls the following joint: "
                     + cnr::control::to_string(this->jointNames()));
  CNR_DEBUG(this->m_logger, "Controller ' "+this->getControllerNamespace()+"' get the setpoint from the topic: '"
                     + m_setpoint_topic_name + "'");
  CNR_RETURN_TRUE(this->m_logger);
}

//!
//! \brief OpenLoopPositionController::doStarting
//! \return
//!
inline bool OpenLoopPositionController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->m_logger);
  m_configured = false;
  CNR_RETURN_TRUE(this->m_logger);
}

//!
//! \brief OpenLoopPositionController::doStopping
//! \return
//!
inline bool OpenLoopPositionController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->m_logger);
  m_configured = false;
  CNR_RETURN_TRUE(this->m_logger);
}

//!
//! \brief OpenLoopPositionController::doUpdate
//! \return
//!
inline bool OpenLoopPositionController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->m_logger);

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->m_logger);
}

//!
//! \brief OpenLoopPositionController::extractJoint
//! \param msg
//! \return
//!
inline bool OpenLoopPositionController::extractJoint(const sensor_msgs::JointState& msg)
{
  size_t cnt = 0;
  auto target = this->getCommandPosition();
  for (size_t iJoint=0; iJoint < msg.name.size(); iJoint++)
  {
    for (size_t iAx=0; iAx < this->jointNames().size(); iAx++)
    {
      if(msg.name.at(iJoint) == this->jointNames().at(iAx))
      {
        if(msg.position.size() > (iJoint))
        {
          eigen_utils::at(target,iAx) = msg.position.at(iJoint);
          cnt++;
        }
        else
        {
          return false;
        }
      }
    }
  }

  bool ok = (cnt == this->nAx());
  if( ok )
  {
    this->setCommandPosition(target);
  }
  else
  {
    CNR_FATAL(this->m_logger, this->getControllerNamespace() + " command message dimension is wrong. found "+std::to_string(cnt)+" over "+std::to_string(this->nAx())+" joints");
    for (size_t iAx=0; iAx < this->jointNames().size(); iAx++)
    {
      CNR_FATAL(this->m_logger, this->getControllerNamespace() + " controlled joints: "+this->jointNames().at(iAx));
    }
    CNR_FATAL(this->m_logger, this->getControllerNamespace() + " received message\n"<< msg);
  }
  return ok;
}


//!
inline void OpenLoopPositionController::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  CNR_TRACE_START_THROTTLE(this->m_logger, 1.0);
  if(extractJoint(*msg))
  {
    m_configured = true;
  }

  CNR_RETURN_OK_THROTTLE(this->m_logger, void(), 1.0);
}

}
}
