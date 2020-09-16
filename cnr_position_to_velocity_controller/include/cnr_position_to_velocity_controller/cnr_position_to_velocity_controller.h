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

class PositionToVelocityController :
    public cnr_controller_interface::JointCommandController<hardware_interface::VelocityJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);


protected:
  cnr::control::PositionToVelocityControllerMath ctrl;
};

class PositionToVelocityControllerFfw :
    public cnr_controller_interface::JointCommandController<hardware_interface::PosVelEffJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);


protected:
  cnr::control::PositionToVelocityControllerMath  ctrl;


};

}  // namespace control
}  // namespace cnr

#endif  // CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER__H
