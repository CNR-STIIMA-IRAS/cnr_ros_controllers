#pragma once

#ifndef cnr_joint_teleop_controller__cnr_joint_teleop_controller_impl_h
#define cnr_joint_teleop_controller__cnr_joint_teleop_controller_impl_h

#include <eigen_state_space_systems/utils/operations.h>
#include <cnr_joint_teleop_controller/cnr_joint_teleop_controller.h>

namespace eu = eigen_utils;

namespace cnr
{
namespace control
{


/**
 * @brief JointTeleopControllerN<N,MaxN>::JointTeleopController
 */
template<int N,int MaxN>
inline JointTeleopControllerN<N,MaxN>::JointTeleopControllerN()
{
}

/**
 * @brief JointTeleopControllerN<N,MaxN>::doInit
 * @return
 */
template<int N,int MaxN>
inline bool JointTeleopControllerN<N,MaxN>::doInit()
{
  //INIT PUB/SUB
  std::string setpoint_topic_name;
  setpoint_topic_name = this->getControllerNamespace() + "/target_joint_teleop";

  this->template add_subscriber<sensor_msgs::JointState>(
        setpoint_topic_name,5,boost::bind(&JointTeleopControllerN<N,MaxN>::callback,this,_1), false);

  this->setPriority(this->QD_PRIORITY);

  if(!this->getControllerNh().getParam("dump_time", m_dump.dump_time))
  {
    m_dump.dump_time = 50 * this->m_sampling_period;
  }

  m_vel_sp = 0 * this->m_rstate.qd();
  m_pos_sp = this->m_rstate.q();

  m_has_pos_sp = false;
  eu::resize(m_scaling_factor,this->nAx());
  eu::setZero(m_scaling_factor);

  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief JointTeleopControllerN<N,MaxN>::doStarting
 * @param time
 */
template<int N,int MaxN>
inline bool JointTeleopControllerN<N,MaxN>::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  m_pos_sp = this->m_rstate.q();
  m_vel_sp = 0 * this->m_rstate.qd();
  m_dist_to_pos_sp =  0 * this->m_rstate.qd();
  m_vel_sp_last = m_vel_sp;
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief JointTeleopControllerN<N,MaxN>::stopping
 * @param time
 */
template<int N,int MaxN>
inline bool JointTeleopControllerN<N,MaxN>::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief JointTeleopControllerN<N,MaxN>::doUpdate
 * @param time
 * @param period
 * @return
 */
template<int N,int MaxN>
inline bool JointTeleopControllerN<N,MaxN>::doUpdate(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;
  std::lock_guard<std::mutex> lock(m_mtx);
  
  ect::Value<N,MaxN> vel_sp = m_vel_sp;
  ect::Value<N,MaxN> pos_sp = m_pos_sp;
  if(m_has_pos_sp)
  {
    auto dist_to_sp_perc = eu::norm(m_pos_sp - this->m_rstate.q()) / eu::norm(m_dist_to_pos_sp);
    auto dir_to_sp      = eu::normalized(m_pos_sp - this->m_rstate.q());
    vel_sp = eu::norm(m_vel_sp) * dist_to_sp_perc * dir_to_sp;
  }
  else
  {
    vel_sp   = m_vel_sp * m_dump.dumpFactor();
    m_pos_sp = m_pos_sp + vel_sp * period.toSec();
    pos_sp   = m_pos_sp;
    if(this->m_rkin->saturatePosition(pos_sp, &report))
    {
      CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
    }
  }
  this->setCommandPosition( pos_sp );
  this->setCommandVelocity( vel_sp );

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

/**
 * @brief JointTeleopControllerN<N,MaxN>::callback
 * @param msg
 */
template<int N,int MaxN>
inline void JointTeleopControllerN<N,MaxN>::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  if(msg->velocity.size() == msg->name.size())
  {
    std::stringstream report;
    try
    {
      std::lock_guard<std::mutex> lock(m_mtx);
      m_has_pos_sp = false;
      eu::setZero(m_vel_sp);
      for( size_t iJoint=0; iJoint< this->jointNames().size(); iJoint++)
      {
        auto it = std::find(msg->name.begin(), msg->name.end(), this->jointName(iJoint));
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
      
      if(this->m_rkin->saturateSpeed(m_vel_sp,
            this->m_rstate.qd(), this->m_rstate.q(), this->m_sampling_period, 1.0, true, &report ))
      {
         CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
      }
      
      if(m_has_pos_sp)
      {
        if(this->m_rkin->saturatePosition(m_pos_sp, &report))
        {
          CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
        }
        m_dist_to_pos_sp = m_pos_sp - this->m_rstate.q();
        m_vel_sp = eu::dot(m_vel_sp, eu::normalized(m_dist_to_pos_sp) ) * eu::normalized(m_dist_to_pos_sp);
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




#endif
