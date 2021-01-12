#ifndef CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_H
#define CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_H

#include <hardware_interface/joint_state_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace cnr
{
namespace control
{

template<int N, int MaxN=N>
class HomingControllerN :
    public cnr::control::JointCommandController<N, MaxN, 
      hardware_interface::JointStateHandle, hardware_interface::PositionJointInterface>
{
public:
  HomingControllerN() = default;
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);
};

using HomingController  = HomingControllerN<-1, cnr::control::max_num_axes >;
using HomingController1 = HomingControllerN<1>;
using HomingController3 = HomingControllerN<3>;
using HomingController6 = HomingControllerN<6>;
using HomingController7 = HomingControllerN<7>;


}
} 

#include <cnr_homing_controller/internal/cnr_homing_controller_impl.h>

#endif  // CNR_HOMING_CONTROLLER__CNR_HOMING_CONTROLLER_H
