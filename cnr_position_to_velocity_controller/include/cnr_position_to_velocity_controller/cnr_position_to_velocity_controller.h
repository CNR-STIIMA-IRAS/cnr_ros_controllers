#ifndef __cnr_pos_to_vel_control__
#define __cnr_pos_to_vel_control__

#include <cnr_controller_interface/cnr_controller_interface.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_math.h>




namespace cnr
{
namespace control
{

class PositionToVelocityController : public cnr_controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);


protected:

  cnr::control::PositionToVelocityControllerMath ctrl;
  hardware_interface::JointHandle                m_jh;

  double m_pos_cmd;
  double m_vel_cmd;
  double m_eff_cmd;

};

class PositionToVelocityControllerFfw : public cnr_controller_interface::Controller<hardware_interface::PosVelEffJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);


protected:

  cnr::control::PositionToVelocityControllerMath  ctrl;
  hardware_interface::PosVelEffJointHandle        m_jh;

  double m_pos_cmd;
  double m_vel_cmd;
  double m_eff_cmd;

};

}
}

# endif
