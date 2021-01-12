#ifndef robot_state_controller_20190319
#define robot_state_controller_20190319

#include <vector>
#include <string>

#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/time.h>
#include <ros/duration.h>

namespace cnr
{
namespace control
{

template<int N, int MaxN=N>
class RobotStateControllerN:
    public cnr::control::JointController<N,MaxN,
                hardware_interface::JointStateHandle, hardware_interface::JointStateInterface>
{
public:
  virtual bool doInit( );
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);

protected:
  std::vector<std::string>  m_frames;
  std::vector<size_t>       m_base_pub_idx;
  std::vector<size_t>       m_link_pub_idx;
  std::vector<unsigned int> m_frame_idxs;

};

using RobotStateController  = RobotStateControllerN<-1, cnr::control::max_num_axes>;
using RobotStateController1 = RobotStateControllerN<1>;
using RobotStateController3 = RobotStateControllerN<3>;
using RobotStateController6 = RobotStateControllerN<6>;
using RobotStateController7 = RobotStateControllerN<7>;

}
}


#include <robot_state_controller/internal/robot_state_controller_impl.h>


#endif
