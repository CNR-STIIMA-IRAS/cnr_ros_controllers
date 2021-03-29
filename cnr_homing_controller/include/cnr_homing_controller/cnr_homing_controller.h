#ifndef CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_H
#define CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_H

#include <hardware_interface/joint_state_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace cnr
{
namespace control
{

class HomingController :
    public cnr::control::JointCommandController<
                  hardware_interface::JointStateHandle, hardware_interface::PositionJointInterface>
{
public:
  HomingController() = default;
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);
};

}
} 

#include <cnr_homing_controller/internal/cnr_homing_controller_impl.h>

#endif  // CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_H
