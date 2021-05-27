#include <state_space_filters/filtered_values.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_joint_velocity_controller/cnr_joint_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointVelocityController, controller_interface::ControllerBase)

namespace eu = eigen_utils;
namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{

/**
 * @brief JointVelocityController::JointVelocityController
 */
inline JointVelocityController::JointVelocityController()
{
}

/**
 * @brief JointVelocityController::doInit
 * @return
 */
inline bool JointVelocityController::doInit()
{
  //INIT PUB/SUB
  std::string setpoint_topic_name;
  setpoint_topic_name = this->getControllerNamespace() + "/target_joint_teleop";

  this->template add_subscriber<sensor_msgs::JointState>(
        setpoint_topic_name,5,boost::bind(&JointVelocityController::setPointCallback,this,_1), false);

  this->setPriority(this->QD_PRIORITY);

  ect::FilteredVectorXd::Value dead_band;
  ect::FilteredVectorXd::Value saturation;
  ect::FilteredVectorXd::Value init_value;

  dead_band = 0.0 * this->chain().getDQMax();
  saturation = this->chain().getDQMax();
  init_value = dead_band;
  if(!m_vel_fitler_sp.activateFilter ( dead_band, saturation, (10.0 / 2.0 / M_PI), this->m_sampling_period, init_value ))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  m_vel_sp = m_vel_fitler_sp.getUpdatedValue();
  m_pos_sp = this->getPosition();

  m_has_pos_sp = false;
  
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief JointVelocityController::doStarting
 * @param time
 */
inline bool JointVelocityController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  m_pos_sp = this->getPosition();
  m_vel_sp = 0 * this->getVelocity();
  m_dist_to_pos_sp =  0 * this->getVelocity();
  m_vel_sp_last = m_vel_sp;
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief JointVelocityController::stopping
 * @param time
 */
inline bool JointVelocityController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief JointVelocityController::doUpdate
 * @param time
 * @param period
 * @return
 */
inline bool JointVelocityController::doUpdate(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;

  std::lock_guard<std::mutex> lock(m_mtx);
  rosdyn::VectorXd vel_sp = m_vel_sp;
  rosdyn::VectorXd pos_sp = m_pos_sp;
  if(m_has_pos_sp)
  {
    auto dist_to_sp_perc = eu::norm(m_pos_sp - this->getPosition()) / eu::norm(m_dist_to_pos_sp);
    auto dir_to_sp      = eu::normalized(m_pos_sp - this->getPosition());
    vel_sp = eu::norm(vel_sp) * dist_to_sp_perc * dir_to_sp;
  }
  else
  {
    m_vel_fitler_sp.update(vel_sp);
    m_pos_sp = m_pos_sp + m_vel_fitler_sp.getUpdatedValue()* period.toSec();
    pos_sp   = m_pos_sp;
    if(rosdyn::saturatePosition(this->chain(),pos_sp, &report))
    {
      CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
    }
    m_pos_sp  = pos_sp;
  }
  this->setCommandPosition( pos_sp );
  this->setCommandVelocity( vel_sp );

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

/**
 * @brief JointVelocityController::setPointCallback
 * @param msg
 */
inline void JointVelocityController::setPointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  if(msg->velocity.size() == msg->name.size())
  {
    std::stringstream report;
    try
    {
      const std::lock_guard<std::mutex> lock(m_mtx);
      m_has_pos_sp = false;
      eu::setZero(m_vel_sp);
      for( size_t iJoint=0; iJoint< this->jointNames().size(); iJoint++)
      {
        auto it = std::find(msg->name.begin(), msg->name.end(), this->chain().getActiveJointName(iJoint));
        if(it!=msg->name.end())
        {
          size_t iMsg = std::distance(msg->name.begin(), it);
          eu::at(m_vel_sp,iJoint) = msg->velocity.at(iMsg);

          if(!msg->position.empty() && (msg->position.size() == msg->name.size()) )
          {
            eu::at(m_pos_sp,iJoint) = msg->position.at(iMsg);
            m_has_pos_sp = true;
          }
        }
      }
            
      if(m_has_pos_sp)
      {
        if(rosdyn::saturatePosition(this->chain(),m_pos_sp, &report))
        {
          CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
        }
        m_dist_to_pos_sp = m_pos_sp - this->getPosition();
        m_vel_sp = eu::dot(m_vel_sp, eu::normalized(m_dist_to_pos_sp) )*eu::normalized(m_dist_to_pos_sp);
      }
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

