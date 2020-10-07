#ifndef ITIA_HOMING_CONTROLLER_201806061719
#define ITIA_HOMING_CONTROLLER_201806061719

#include <hardware_interface/joint_state_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace cnr
{
namespace control
{

class HomingController :
    public cnr_controller_interface::JointCommandController<hardware_interface::JointStateHandle, hardware_interface::JointCommandInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);


};

}
}

# endif
