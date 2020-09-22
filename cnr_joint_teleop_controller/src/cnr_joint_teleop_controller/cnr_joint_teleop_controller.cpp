#include <cnr_joint_teleop_controller/cnr_joint_teleop_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointTeleopController, controller_interface::ControllerBase)


namespace cnr
{
namespace control
{


/**
 * @brief JointTeleopController::JointTeleopController
 */
JointTeleopController::JointTeleopController()
{
}

/**
 * @brief JointTeleopController::doInit
 * @return
 */
bool JointTeleopController::doInit()
{

  //INIT PUB/SUB
  std::string setpoint_topic_name;
  setpoint_topic_name = getControllerNamespace() + "/target_joint_teleop";

  add_subscriber<sensor_msgs::JointState>(setpoint_topic_name,5,boost::bind(&JointTeleopController::callback,this,_1));

  this->setPriority(QD_PRIORITY);
  CNR_RETURN_TRUE(*m_logger);
}

/**
 * @brief JointTeleopController::doStarting
 * @param time
 */
bool JointTeleopController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger,"Starting Controller");

  CNR_RETURN_TRUE(*m_logger);
}

/**
 * @brief JointTeleopController::stopping
 * @param time
 */
bool JointTeleopController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger,"Stopping Controller");
  CNR_RETURN_TRUE(*m_logger);
}

/**
 * @brief JointTeleopController::doUpdate
 * @param time
 * @param period
 * @return
 */
bool JointTeleopController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  return true;
}

/**
 * @brief JointTeleopController::callback
 * @param msg
 */
void JointTeleopController::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    for( size_t i = 0; i< msg->name.size( ); i++)
    {
      for( size_t j=0; j< jointNames().size(); j++)
      {
        if( msg->name.at(i)==jointNames().at(j) )
        {
          if(msg->velocity.size() > 0 )
          {
            this->setCommandVelocity(!std::isnan(msg->velocity[i]) ? msg->velocity[i] : this->getCommandVelocity(j), j);
          }
          else
          {
            CNR_ERROR_THROTTLE(*m_logger, 5.0, "The message is broken.. No velocity stored " << *msg  );
          }
        }
      }
    }
  }
  catch(...)
  {
    ROS_WARN("[ %s ] Something wrong in Target Callback",  getControllerNamespace().c_str());
    this->setCommandPosition( this->q() );
    this->setCommandVelocity( this->qd() * 0.0 );
  }
  return;
}

}
}

