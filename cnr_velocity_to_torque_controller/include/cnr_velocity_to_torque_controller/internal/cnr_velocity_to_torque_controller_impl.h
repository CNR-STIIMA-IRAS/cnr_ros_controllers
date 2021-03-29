#pragma once // qtcreator, workaraound ctidy

#ifndef cnr_velocity_to_torque_controller__cnr_velocity_to_torque_controller_impl__h
#define cnr_velocity_to_torque_controller__cnr_velocity_to_torque_controller_impl__h

#include <rosparam_utilities/rosparam_utilities.h>
#include <state_space_ros/ros_params.h>
#include <cnr_velocity_to_torque_controller/cnr_velocity_to_torque_controller.h>

namespace cnr
{
namespace control
{

//!
inline bool VelocityToTorqueController::doInit( )
{
  CNR_TRACE_START(this->logger());
  std::string setpoint_topic_name;

  if(!this->getControllerNh().getParam("setpoint_topic_name", setpoint_topic_name))
  {
    CNR_ERROR(this->logger(), this->getControllerNamespace() + "/setpoint_topic_name does not exist");
    CNR_RETURN_FALSE(this->logger());
  }
  this->template add_subscriber<sensor_msgs::JointState>(setpoint_topic_name,1,
                                    boost::bind(&VelocityToTorqueController::callback, this, _1));

  m_use_target_torque = false;
  if(!this->getControllerNh().getParam("use_target_torque", m_use_target_torque))
  {
    CNR_WARN(this->logger(), this->getControllerNamespace() + "/use_target_torque does not exist, set FALSE");
  }

  std::string what;
  std::string msg;
  int ok = eigen_control_toolbox::setMatricesFromParam(m_filter,this->getControllerNh(), "vel_filter",msg);
  if(ok==-1)
  {
    CNR_ERROR(this->logger(), this->getControllerNamespace() + "/vel_filter: " << what);
    return false;
  }
  else if(ok==0)
  {
    CNR_WARN(this->logger(), this->getControllerNamespace() + "/vel_filter: " << what);
  }

  ok = eigen_control_toolbox::setMatricesFromParam(m_target_filter,this->getControllerNh(), "target_vel_filter",msg);
  if(ok==-1)
  {
    CNR_ERROR(this->logger(), this->getControllerNamespace() + "/target_vel_filter: " << what);
    return false;
  }
  else if(ok==0)
  {
    CNR_WARN(this->logger(), this->getControllerNamespace() + "/target_vel_filter: " << what);
  }

  ok = eigen_control_toolbox::setMatricesFromParam(m_controller,this->getControllerNh(), "controller",msg);
  if(ok==-1)
  {
    CNR_ERROR(this->logger(), this->getControllerNamespace() + "/controller: " << what);
    return false;
  }
  else if(ok==0)
  {
    CNR_WARN(this->logger(), this->getControllerNamespace() + "/controller: " << what);
  }

  ok = eigen_control_toolbox::setMatricesFromParam(m_integral_controller,this->getControllerNh(),"integral_controller",msg);
  if(ok==-1)
  {
    CNR_ERROR(this->logger(), this->getControllerNamespace() + "/integral_controller: " << what);
    return -1;
  }
  else if(ok==0)
  {
    CNR_WARN(this->logger(), this->getControllerNamespace() + "/integral_controller: " << what);
  }

  eigen_utils::resize(m_antiwindup_gain , this->nAx(), this->nAx() );eigen_utils::setDiagonal(m_antiwindup_gain, 1);
  eigen_utils::resize(m_pos_deadband    , this->nAx() );eigen_utils::setZero(m_pos_deadband    );
  eigen_utils::resize(m_vel_cmd         , this->nAx() );eigen_utils::setZero(m_vel_cmd         );
  eigen_utils::resize(m_eff_cmd         , this->nAx() );eigen_utils::setZero(m_eff_cmd         );
  eigen_utils::resize(m_antiwindup      , this->nAx() );eigen_utils::setZero(m_antiwindup      );
  eigen_utils::resize(m_max_effort      , this->nAx() );eigen_utils::setZero(m_max_effort      );
  eigen_utils::resize(m_target_vel      , this->nAx() );eigen_utils::setZero(m_target_vel      );
  eigen_utils::resize(m_target_eff      , this->nAx() );eigen_utils::setZero(m_target_eff      );

  if(!rosparam_utilities::getParam(this->getControllerNh(), "antiwindup_ratio", m_antiwindup_gain, what, &m_antiwindup_gain))
  {
    return false;
  }

  if(!rosparam_utilities::getParam(this->getControllerNh(), "maximum_torque", m_max_effort, what, &m_max_effort))
  {
    return false;
  }

  m_well_init = true;

  this->setPriority(this->QD_PRIORITY);

  return true;
}

//!
inline bool VelocityToTorqueController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  m_configured = false;

  auto fb_vel = this->getPosition();
  auto fb_eff = this->getEffort();

  m_target_vel = fb_vel;
  eigen_utils::setZero(m_target_eff);

  auto init_vel = fb_vel;
  m_filter.setStateFromLastIO(init_vel, init_vel);

  init_vel = m_target_vel;
  m_target_filter.setStateFromLastIO(init_vel, init_vel);

  auto init_eff   = (m_use_target_torque) ? fb_eff - m_target_eff : fb_eff;
  auto init_error = m_target_filter.y() - m_filter.y();

  m_controller.setStateFromLastIO(init_error, init_eff);
  m_integral_controller.setStateFromLastIO(init_error, init_eff);
  eigen_utils::setZero(m_antiwindup);

  CNR_RETURN_TRUE(this->logger());
}

//!
inline bool VelocityToTorqueController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  m_configured = false;
  eigen_utils::setZero(m_vel_cmd);
  CNR_RETURN_TRUE(this->logger());
}

//!
inline bool VelocityToTorqueController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    if (!m_configured)
    {
      m_target_vel = this->getVelocity();
      eigen_utils::setZero(m_target_eff);
    }

    auto filter_output = m_filter.update(this->getVelocity());
    auto target_filter_output = (m_configured)
                              ? m_target_filter.update(m_target_vel)
                              : m_target_filter.update(m_target_filter.y());

    auto controller_input = target_filter_output - filter_output; //controller error
    auto controller_output = m_controller.update(controller_input);

    auto integral_controller_input = controller_input + m_antiwindup_gain * m_antiwindup; //integral controller error
    auto integral_controller_output = m_integral_controller.update(integral_controller_input);

    m_vel_cmd = target_filter_output;
    m_eff_cmd = controller_output + integral_controller_output;

    if(m_use_target_torque)
    {
      m_eff_cmd += m_target_eff;
    }

    for(int i=0;i<int(this->nAx());i++)
    {
      if (eigen_utils::at(m_eff_cmd,i) > eigen_utils::at(m_max_effort,i) )
      {
        eigen_utils::at(m_antiwindup,i) = eigen_utils::at(m_max_effort,i) - eigen_utils::at(m_eff_cmd,i);
        eigen_utils::at(m_eff_cmd   ,i) = eigen_utils::at(m_max_effort,i);
      }
      else if (eigen_utils::at(m_eff_cmd,i) < -eigen_utils::at(m_max_effort,i) )
      {
        eigen_utils::at(m_antiwindup,i) = -eigen_utils::at(m_max_effort,i) - eigen_utils::at(m_eff_cmd,i);
        eigen_utils::at(m_eff_cmd   ,i) = -eigen_utils::at(m_max_effort,i);
      }
      else
      {
        eigen_utils::setZero(m_antiwindup);
      }
    }
    this->setCommandEffort(m_eff_cmd);
  }
  catch (...)
  {
    CNR_ERROR(this->logger(), "Something wrong");
    eigen_utils::setZero(m_eff_cmd);
    this->setCommandEffort(m_eff_cmd);
  }
  CNR_RETURN_TRUE(this->logger());
}


//!
inline bool VelocityToTorqueController::extractJoint(
    const sensor_msgs::JointState& msg, const std::vector<std::string>& names,
    rosdyn::VectorXd& vel, rosdyn::VectorXd& eff)
{
  if(msg.velocity.size()!=msg.name.size())
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
    eigen_utils::at(vel,i) = msg.velocity.at(iJoint);
    eigen_utils::at(eff,i) = msg.effort.size() == msg.name.size() ? msg.effort.at(iJoint) : 0 ;
  }

  return true;
}

//!
inline void VelocityToTorqueController::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (extractJoint(*msg, this->jointNames(), m_target_vel, m_target_eff))
  {
    m_configured = true;
  }
  else
  {
    CNR_ERROR(this->logger(), this->getControllerNamespace() + " target message dimension is wrong");
  }
  return;
}

}
}

#endif  // cnr_velocity_to_torque_controller__cnr_velocity_to_torque_controller_impl__h
