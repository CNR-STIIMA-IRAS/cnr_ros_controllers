#ifndef CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER__H
#define CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER__H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>


namespace cnr
{
namespace control
{

template<int N,int MaxN=N>
class OpenLoopEffortControllerN :
    public cnr::control::JointCommandController<N,MaxN,
                     hardware_interface::JointHandle, hardware_interface::EffortJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  cnr::control::Vector  m_eff_cmd;
  cnr::control::Vector  m_max_effort;

  std::string     m_setpoint_topic_name;
  bool            m_configured;

  void callback(const boost::shared_ptr<const sensor_msgs::JointState> & msg);
  bool extractJoint(const sensor_msgs::JointState& msg);

  const std::string SP_TOPIC_ID = "sp";

};

using OpenLoopEffortController  = OpenLoopEffortControllerN<-1, cnr::control::max_num_axes>;
using OpenLoopEffortController1 = OpenLoopEffortControllerN<1>;
using OpenLoopEffortController3 = OpenLoopEffortControllerN<3>;
using OpenLoopEffortController6 = OpenLoopEffortControllerN<6>;
using OpenLoopEffortController7 = OpenLoopEffortControllerN<7>;
using OpenLoopEffortController8 = OpenLoopEffortControllerN<8>;


}  // namespace control
}  // namespace cnr

#include <cnr_open_loop_effort_controller/internal/cnr_open_loop_effort_controller_impl.h>

#endif  // CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER__H
