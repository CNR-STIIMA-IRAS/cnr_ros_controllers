#ifndef CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER__H
#define CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER__H

#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_math.h>

namespace cnr
{
namespace control
{

template<class T>
class PositionToVelocityControllerBase:
    public cnr_controller_interface::JointCommandController<T>
{
public:
  virtual bool doInit()
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
                                          boost::bind(&PositionToVelocityControllerBase::callback,this,_1));

    m_target_pos = this->q(0);
    m_target_vel = 0;
    m_target_eff = 0;

    if(!ctrl.init(this->getRootNh(), this->getControllerNh()))
    {
      CNR_RETURN_FALSE(this->logger(), "Math ctrl of the PositionToVelocityController failed in initialization.");
    }
    m_configured = false;
    this->setPriority(this->QD_PRIORITY);
    CNR_RETURN_TRUE(this->logger());
  }

  virtual bool doStarting(const ros::Time& time)
  {
    CNR_TRACE_START(this->logger());
    m_target_pos = this->q(0);
    m_target_vel = 0;
    m_target_eff = 0;
    m_configured = false;
    ctrl.starting(time, m_target_pos, m_target_vel);
    CNR_RETURN_TRUE(this->logger());
  }

  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period)
  {
    try
    {
      if(!m_configured)
      {
        m_target_pos = this->q(0);
        m_target_vel = 0;
        ctrl.update(time, period, nullptr, nullptr, nullptr, nullptr, this->q(0), this->qd(0));
      }
      else
      {
        ctrl.update(time, period, &m_target_pos, &m_target_vel, &m_target_eff, &m_last_sp_time, this->q(0), this->qd(0));
      }
      this->setCommandVelocity(ctrl.getVelCmd(), 0);
    }
    catch (...)
    {
      this->setCommandVelocity(0,0);
      CNR_RETURN_FALSE_THROTTLE(this->logger(), 2.0, "Exception!");
    }
    CNR_RETURN_TRUE(this->logger());
  }

  virtual bool doStopping(const ros::Time& time)
  {
    CNR_TRACE_START(this->logger());
    m_configured = false;
    ctrl.stopping(time);
    this->setCommandVelocity(0,0);
    CNR_RETURN_TRUE(this->logger());
  }


protected:
  cnr::control::PositionToVelocityControllerMath ctrl;
  double  m_target_pos;
  double  m_target_vel;
  double  m_target_eff;
  bool    m_configured;
  double  m_last_sp_time;

  void callback(const sensor_msgs::JointStateConstPtr msg)
  {
    if(extractJoint(*msg, this->jointName(0), m_target_pos, m_target_vel, m_target_eff))
    {
      m_configured = true;
      m_last_sp_time = msg->header.stamp.toSec();
    }
    else
    {
      CNR_ERROR(this->logger(), " target message dimension is wrong");
      CNR_ERROR(this->logger(), " msg received: " << *msg);
    }
    return;
  }

  bool extractJoint(const sensor_msgs::JointState msg, const std::string name, double& pos, double& vel, double& eff)
  {
    for(unsigned int iJoint = 0; iJoint < msg.name.size(); iJoint++)
    {
      if(!msg.name.at(iJoint).compare(name))
      {
        if(msg.position.size() > (iJoint))
          pos = msg.position.at(iJoint);
        else
          return false;

        if(msg.velocity.size() > (iJoint))
          vel = msg.velocity.at(iJoint);
        else
          return false;

        if(msg.effort.size() > (iJoint))
          eff = msg.effort.at(iJoint);
        else
          return false;

        return true;
      }
    }
    return false;
  }
};

typedef PositionToVelocityControllerBase<hardware_interface::VelocityJointInterface> PositionToVelocityController;


class PositionToVelocityControllerFfw :
    public PositionToVelocityControllerBase<hardware_interface::PosVelEffJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

};

}  // namespace control
}  // namespace cnr

#endif  // CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER__H
