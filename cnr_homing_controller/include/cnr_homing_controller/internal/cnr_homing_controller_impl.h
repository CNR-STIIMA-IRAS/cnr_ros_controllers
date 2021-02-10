#pragma once  // workaraound clang-tidy

#ifndef CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_IMPL_H
#define CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_IMPL_H

#include <cnr_homing_controller/cnr_homing_controller.h>

namespace cnr
{
namespace control
{

template<int N, int MaxN>
inline bool HomingControllerN<N,MaxN>::doInit()
{
  CNR_TRACE_START(this->logger());
  if(this->nAx() > 1)
    CNR_RETURN_FALSE(this->logger(),
     "Homing controller is designed for just 1 joint, while "+std::to_string(this->nAx())+" joints configured. Abort.");
  CNR_INFO(this->logger(), "Init homing of joint " << cnr::control::to_string(this->m_chain.getActiveJointsName()));
  this->setPriority(this->Q_PRIORITY);
  CNR_RETURN_TRUE(this->logger());
}

template<int N, int MaxN>
inline bool HomingControllerN<N,MaxN>::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  CNR_INFO(this->logger(), "Start homing of joint " << cnr::control::to_string(this->m_chain.getActiveJointsName()));
  CNR_RETURN_TRUE(this->logger());
}

template<int N, int MaxN>
inline bool HomingControllerN<N,MaxN>::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  CNR_INFO(this->logger(), "Stop homing of joint " << cnr::control::to_string(this->m_chain.getActiveJointsName()));
  CNR_RETURN_TRUE(this->logger());
}

template<int N, int MaxN>
inline bool HomingControllerN<N,MaxN>::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_RETURN_TRUE(this->logger());
}

}
}

#endif // CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_IMPL_H
