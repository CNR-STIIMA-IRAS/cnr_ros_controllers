#pragma once

#ifndef CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER_IMPL__H
#define CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER_IMPL__H

#include <eigen_matrix_utils/overloads.h>
#include <cnr_open_loop_effort_controller/cnr_open_loop_effort_controller.h>

namespace cnr
{
namespace control
{

//!
//! \brief OpenLoopEffortController::doInit
//! \return
//!
inline bool OpenLoopEffortController::doInit( )
{
  CNR_TRACE_START(this->logger());
  if (!this->getControllerNh().getParam("setpoint_topic_name", m_setpoint_topic_name))
  {
    CNR_RETURN_FALSE(this->logger(),this->getControllerNamespace()+"/'setpoint_topic_name' does not exist");
  }
  this->template add_subscriber<sensor_msgs::JointState>(m_setpoint_topic_name, 1,
        boost::bind(&OpenLoopEffortController::callback,this,_1));

  CNR_DEBUG(this->logger(), "Controller ' " +this->getControllerNamespace()+"' controls the following joint: "
                              + cnr::control::to_string(this->jointNames()));
  CNR_DEBUG(this->logger(), "Controller ' " +this->getControllerNamespace()+"' get the setpoint from the topic: '"
                              + m_setpoint_topic_name + "'");

  std::vector<double> max_effort;
  if (!this->getControllerNh().getParam("maximum_torque",max_effort))
  {
    CNR_WARN(this->logger(), "no maximum_torque specified");
    eigen_utils::resize(m_max_effort, this->nAx());
    eigen_utils::setZero(m_max_effort);
  }
  if(max_effort.size() != this->nAx() )
  {
    CNR_RETURN_FALSE( this->logger(), "maximum_torque mismatches dimension!");
  }
  eigen_utils::resize(m_eff_cmd,this->nAx());
  eigen_utils::setZero(m_eff_cmd);

  m_max_effort = Eigen::Map<rosdyn::VectorXd>(max_effort.data(), this->nAx());

  this->setPriority(this->Q_PRIORITY);
  CNR_RETURN_TRUE(this->logger());
}

//!
//! \brief OpenLoopEffortController::doStarting
//! \return
//!
inline bool OpenLoopEffortController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  m_configured = false;
  CNR_RETURN_TRUE(this->logger());
}

//!
//! \brief OpenLoopEffortController::doStopping
//! \return
//!
inline bool OpenLoopEffortController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  m_configured = false;
  CNR_RETURN_TRUE(this->logger());
}

//!
//! \brief OpenLoopEffortController::doUpdate
//! \return
//!
inline bool OpenLoopEffortController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE(this->logger(), 10.0);
  try
  {
    for(size_t iAx=0; iAx<this->nAx(); iAx++)
    {
      if (m_eff_cmd(iAx) > m_max_effort(iAx))
      {
        m_eff_cmd(iAx) = m_max_effort(iAx);
      }
      else if (m_eff_cmd(iAx)<-m_max_effort(iAx))
      {
        m_eff_cmd(iAx) = -m_max_effort(iAx);
      }
    }
    this->setCommandEffort(m_eff_cmd);
  }
  catch (...)
  {
    CNR_WARN(this->logger(), "something wrong!");
    m_eff_cmd.setZero();
    this->setCommandEffort(m_eff_cmd);
  }
  CNR_RETURN_TRUE_THROTTLE(this->logger(), 10.0);
}


//!
//! \brief OpenLoopEffortController::extractJoint
//! \param msg
//! \return
//!
inline bool OpenLoopEffortController::extractJoint(const sensor_msgs::JointState& msg)
{
  size_t cnt = 0;
  auto target = this->getCommandEffort();
  for (size_t iJoint=0; iJoint < msg.name.size(); iJoint++)
  {
    for (size_t iAx=0; iAx < this->jointNames().size(); iAx++)
    {
      if (msg.name.at(iJoint) == this->jointNames().at(iAx))
      {
        if (msg.effort.size() > (iJoint))
        {
          eigen_utils::at(target,iAx) = msg.effort.at(iJoint);
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
  return ok;
}

//!
//! \brief OpenLoopEffortController::callback
//! \param msg
//!
inline void OpenLoopEffortController::callback(const boost::shared_ptr<sensor_msgs::JointState const>& msg)
{
  CNR_TRACE_START_THROTTLE(this->logger(), 5.0);
  if (extractJoint(*msg))
  {
    m_configured = true;
  }
  else
  {
    CNR_FATAL(this->logger(),this->getControllerNamespace()+" command message dimension is wrong.");
  }
  CNR_RETURN_OK_THROTTLE(this->logger(), void(), 5.0);
}

}
}
#endif // CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER_IMPL__H

