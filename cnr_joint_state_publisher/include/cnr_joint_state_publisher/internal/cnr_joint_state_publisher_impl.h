#pragma once // workaorund clang-tidy

#ifndef CNR_JOINT_STATE_PUBLISHER__NR_JOINT_STATE_PUBLISHER_IMPL__H
#define CNR_JOINT_STATE_PUBLISHER__NR_JOINT_STATE_PUBLISHER_IMPL__H

#include <cnr_joint_state_publisher/cnr_joint_state_publisher.h>

namespace cnr
{
namespace control
{

template<int N, int MaxN>
inline JointStatePublisherN<N,MaxN>::~JointStatePublisherN()
{
  CNR_TRACE_START(this->logger());
  if (!this->isStopped())
  {
    this->stopping(ros::Time::now());
  }
}

template<int N, int MaxN>
inline bool JointStatePublisherN<N,MaxN>::doInit()
{
  CNR_TRACE_START(this->logger());
  if (this->nAx()==0)
  {
    CNR_RETURN_FALSE(this->logger(), "The number of controlled axes is 0. Check the configuration. Abort");
  }
  m_pub_handle = this->template add_publisher<sensor_msgs::JointState>("joint_states", 1);

  m_msg.reset(new sensor_msgs::JointState());
  m_msg->position.resize(this->nAx(), 0);
  m_msg->velocity.resize(this->nAx(), 0);
  m_msg->effort.resize(this->nAx(), 0);
  m_msg->name = this->jointNames();

  CNR_TRACE(this->logger(), "Published Topic '" + this->getPublisher(m_pub_handle)->getTopic()
                     + "', axis names: " + cnr::control::to_string(this->jointNames())
            + " n. axes: " + std::to_string(this->nAx()));
  CNR_RETURN_TRUE(this->logger());
}

template<int N, int MaxN>
inline bool JointStatePublisherN<N,MaxN>::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  CNR_RETURN_TRUE(this->logger());
}

template<int N, int MaxN>
inline bool JointStatePublisherN<N,MaxN>::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    for(std::size_t iAx = 0; iAx<this->m_chain.getActiveJointsNumber(); iAx++)
    {
      m_msg->name    .at(iAx) = this->m_chain.getJointName(iAx);
      m_msg->position.at(iAx) = this->getPosition(iAx);
      m_msg->velocity.at(iAx) = this->getVelocity(iAx);
      m_msg->effort  .at(iAx) = this->getEffort(iAx);
    }
    m_msg->header.stamp = ros::Time::now();
    if(!this->publish(m_pub_handle, *m_msg))
    {
      CNR_RETURN_FALSE(this->logger());
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(this->logger(), "Exception caught" + std::string(e.what()));
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<int N, int MaxN>
inline bool JointStatePublisherN<N,MaxN>::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  this->getPublisher(m_pub_handle)->shutdown();
  CNR_RETURN_TRUE(this->logger());
}


}
}

#endif  // CNR_JOINT_STATE_PUBLISHER__NR_JOINT_STATE_PUBLISHER_IMPL__H