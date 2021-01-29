#pragma once // workdaraound qtcreator clang-tidy

#ifndef CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_IMPL_H
#define CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_IMPL_H

#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_controller.h>

namespace ect = eigen_control_toolbox;
namespace eu  = eigen_utils;

namespace cnr
{
namespace control
{

template<int N,int MaxN, class H, class T>
inline bool PositionToVelocityControllerBaseN<N,MaxN,H,T>::doInit()
{
  CNR_TRACE_START(this->logger());
  if(this->nAx()>1)
  {
    CNR_RETURN_FALSE(this->logger(), "The controller is designed to control only one joint.");
  }

  std::string setpoint_topic_name;
  //std::string feedforward_topic_name;

  if(!this->getControllerNh().getParam("setpoint_topic_name", setpoint_topic_name))
  {
    CNR_ERROR(this->logger(), this->getControllerNamespace() + "/'setpoint_topic_name' does not exist");
    CNR_ERROR(this->logger(), "ERROR DURING INITIALIZATION CONTROLLER "<< this->getControllerNamespace());
    CNR_RETURN_FALSE(this->logger());
  }

  this->template add_subscriber<sensor_msgs::JointState>(setpoint_topic_name, 1,
              boost::bind(&PositionToVelocityControllerBaseN<N,MaxN,H,T>::callback,this,_1));

  m_target_pos = this->m_rstate.q();
  eu::setZero(m_target_vel);
  eu::setZero(m_target_eff);

  typename ect::Value<N,MaxN> speed_limit;
  eu::copy(speed_limit, this->speedLimit());
  std::string what;
  int ok = ctrl.init(this->getControllerNh(), speed_limit,what);
  if(ok==-1)
  {
    CNR_RETURN_FALSE(this->logger(),
      "Math ctrl of the PositionToVelocityController failed in initialization:\n\t" + what);
  }
  else if(ok==0)
  {
    CNR_WARN(this->logger(), what);
  }
  m_configured = false;
  this->setPriority(this->QD_PRIORITY);
  CNR_RETURN_TRUE(this->logger());
}

template<int N,int MaxN, class H, class T>
inline bool PositionToVelocityControllerBaseN<N,MaxN,H,T>::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  m_target_pos = this->m_rstate.q();
  eu::setZero(m_target_vel);
  eu::setZero(m_target_eff);
  m_configured = false;
  ctrl.starting(m_target_pos, m_target_vel);
  CNR_RETURN_TRUE(this->logger());
}

template<int N,int MaxN, class H, class T>
inline bool PositionToVelocityControllerBaseN<N,MaxN,H,T>::doUpdate(const ros::Time& time, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    if(!m_configured)
    {
      m_target_pos = this->m_rstate.q();
      eu::setZero(m_target_vel);
      ctrl.update(time, nullptr, nullptr, nullptr, nullptr, this->m_rstate.q(), this->m_rstate.qd());
    }
    else
    {
      ctrl.update(time, &m_target_pos, &m_target_vel, &m_target_eff, &m_last_sp_time,
                  this->m_rstate.q(), this->m_rstate.qd());
    }
    this->setCommandVelocity(ctrl.getVelCmd());
  }
  catch (...)
  {
    this->setCommandVelocity(0,0);
    CNR_RETURN_FALSE_THROTTLE(this->logger(), 2.0, "Exception!");
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<int N,int MaxN, class H, class T>
inline bool PositionToVelocityControllerBaseN<N,MaxN,H,T>::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  m_configured = false;
  ctrl.stopping();
  this->setCommandVelocity(0,0);
  CNR_RETURN_TRUE(this->logger());
}

template<int N,int MaxN, class H, class T>
inline void PositionToVelocityControllerBaseN<N,MaxN,H,T>::callback(const sensor_msgs::JointStateConstPtr msg)
{
  if(this->extractJoint(*msg, this->jointNames(), m_target_pos, m_target_vel, m_target_eff))
  {
    m_configured = true;
    m_last_sp_time = msg->header.stamp.toSec();
  }
  else
  {
    CNR_ERROR(this->logger(), " target message dimension is wrong");
    CNR_ERROR(this->logger(), " Joint Controlled name: " << this->jointName(0));
    CNR_ERROR(this->logger(), " msg received: " << *msg);
  }
  return;
}

template<int N,int MaxN, class H, class T>
inline bool PositionToVelocityControllerBaseN<N,MaxN,H,T>::extractJoint(
    const sensor_msgs::JointState msg, const std::vector<std::string>& names,
    ect::Value<N,MaxN>& pos, ect::Value<N,MaxN>& vel, ect::Value<N,MaxN>& eff)
{
  if(msg.position.size()!=msg.name.size())
  {
    return false;
  }

  for(size_t i=0;i<names.size(); i++)
  {
    std::vector<std::string>::const_iterator it = std::find(msg.name.begin(), msg.name.end(), names.at(i));
    if(it == msg.name.end())
    {
      return false;
    }

    size_t iJoint = std::distance(msg.name.begin(), it);
    eu::at(pos,i) = msg.position.at(iJoint);
    eu::at(vel,i) = msg.velocity.size() == msg.name.size() ? msg.velocity.at(iJoint) : 0 ;
    eu::at(eff,i) = msg.effort.size() == msg.name.size() ? msg.effort.at(iJoint) : 0 ;
  }

  return true;
}

































/**
 * @brief PositionToVelocityControllerFfw::doInit
 * @return
 */
template<int N, int MaxN>
inline bool PositionToVelocityControllerFfwN<N,MaxN>::doInit()
{
  CNR_TRACE_START(this->logger());
  if(this->PositionToVelocityControllerFfwBaseN<N,MaxN>::doInit())
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

template<int N, int MaxN>
inline bool PositionToVelocityControllerFfwN<N,MaxN>::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(this->PositionToVelocityControllerFfwBaseN<N,MaxN>::doStarting(time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

template<int N, int MaxN>
inline bool PositionToVelocityControllerFfwN<N,MaxN>::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(this->PositionToVelocityControllerFfwBaseN<N,MaxN>::doStopping(time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  this->setCommandVelocity(0,0);
  this->setCommandEffort(0,0);
  CNR_RETURN_TRUE(this->logger());
}

template<int N, int MaxN>
inline bool PositionToVelocityControllerFfwN<N,MaxN>::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    if(this->PositionToVelocityControllerFfwBaseN<N,MaxN>::doUpdate(time, period))
    {
      CNR_RETURN_FALSE(this->logger());
    }
    this->setCommandPosition(this->ctrl.getPosCmd());
    this->setCommandVelocity(this->ctrl.getVelCmd());
    this->setCommandEffort(this->ctrl.getEffCmd());
  }
  catch (...)
  {
    auto zero = this->ctrl.getVelCmd();
    eu::setZero(zero);
    this->setCommandVelocity(zero);
    this->setCommandEffort(zero);
    CNR_RETURN_FALSE_THROTTLE(this->logger(), 2.0,"something wrong: Controller '"+this->getControllerNamespace()+"'");
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}






}  // namespace control
}  // namespace cnr
#endif  // CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_IMPL_H
