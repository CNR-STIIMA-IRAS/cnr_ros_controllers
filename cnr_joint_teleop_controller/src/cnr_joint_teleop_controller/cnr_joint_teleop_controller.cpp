#include <cnr_logger/cnr_logger_macros.h>
#include <name_sorting/name_sorting.h>
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

  add_subscriber<sensor_msgs::JointState>(
        setpoint_topic_name,5,boost::bind(&JointTeleopController::callback,this,_1), false);

  this->setPriority(QD_PRIORITY);

  if(!this->getControllerNh().getParam("dump_time", m_dump.dump_time))
  {
    m_dump.dump_time = 50 * this->m_sampling_period;
  }

  m_vel_sp = 0 * this->qd();
  m_pos_sp = this->q();

  m_has_pos_sp = false;
  m_scaling_factor = 0 * this->qd();

  CNR_RETURN_TRUE(*m_logger);
}

/**
 * @brief JointTeleopController::doStarting
 * @param time
 */
bool JointTeleopController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger,"Starting Controller");
  m_pos_sp = this->q();
  m_vel_sp = 0 * this->qd();
  m_dist_to_pos_sp =  0 * this->qd();
  m_vel_sp_last = m_vel_sp;
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
  std::stringstream report;
  std::lock_guard<std::mutex> lock(m_mtx);
  
  Eigen::VectorXd vel_sp = m_vel_sp;
  Eigen::VectorXd pos_sp = m_pos_sp;
  if(m_has_pos_sp)
  {
    double dist_to_sp_perc = (m_pos_sp - this->q()).norm() / m_dist_to_pos_sp.norm();
    Eigen::VectorXd dir_to_sp = (m_pos_sp - this->q()).normalized();
    vel_sp = m_vel_sp.norm() * dist_to_sp_perc * dir_to_sp;
  }
  else
  {
    vel_sp = m_vel_sp * m_dump.dumpFactor();
    pos_sp = this->q() + vel_sp * period.toSec();
    if(this->m_kin->saturatePosition(pos_sp, &report))
    {
      CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
    }
  }

  this->setCommandPosition( pos_sp );
  this->setCommandVelocity( vel_sp );

  return true;
}

/**
 * @brief JointTeleopController::callback
 * @param msg
 */
void JointTeleopController::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  if(msg->velocity.size() == msg->name.size())
  {
    std::stringstream report;
    try
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      m_has_pos_sp = false;
      m_vel_sp.setZero();
      for( size_t iJoint=0; iJoint< this->jointNames().size(); iJoint++)
      {
        auto it = std::find(msg->name.begin(), msg->name.end(), this->jointName(iJoint));
        if(it!=msg->name.end())
        {
          size_t iMsg = std::distance(msg->name.begin(), it);
          m_vel_sp(iJoint) = msg->velocity.at(iMsg);

          if(!msg->position.empty() && (msg->position.size() == msg->name.size()) )
          {
            m_pos_sp(iJoint) = msg->position.at(iMsg);
            m_has_pos_sp = true;
          }
        }
      }
      
      if(this->m_kin->saturateSpeed(m_vel_sp, this->qd(), this->q(), this->m_sampling_period, 1.0, true, &report ))
      {
         CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
      }
      
      if(m_has_pos_sp)
      {
        if(this->m_kin->saturatePosition(m_pos_sp, &report))
        {
          CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
        }
        m_dist_to_pos_sp = m_pos_sp - this->q();
        m_vel_sp = m_vel_sp.dot( m_dist_to_pos_sp.normalized() ) * m_dist_to_pos_sp.normalized();
      }
      m_dump.tick();
    }
    catch(...)
    {
      CNR_WARN_THROTTLE(this->logger(), 5.0, "Something wrong in Target Callback");
    }
  }
  return;
}



}
}




