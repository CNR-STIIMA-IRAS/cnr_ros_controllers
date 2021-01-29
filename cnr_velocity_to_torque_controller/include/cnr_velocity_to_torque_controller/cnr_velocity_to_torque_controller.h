#ifndef cnr_vel_to_torque_control__20188101642
#define cnr_vel_to_torque_control__20188101642

#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <ros/time.h>
#include <ros/duration.h>
#include <sensor_msgs/JointState.h>

#include <subscription_notifier/subscription_notifier.h>
#include <eigen_state_space_systems/discrete_state_space_systems.h>
#include <eigen_state_space_systems/controllers/controllers.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{

template<int N,int MaxN=N>
class VelocityToTorqueControllerN :
    public cnr::control::JointCommandController<N, MaxN,
                  hardware_interface::JointHandle, hardware_interface::EffortJointInterface>
{
public:
  bool doInit( );
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  bool m_well_init;
  bool m_use_feedback;

  ect::Controller<N,MaxN> m_controller;
  ect::Controller<N,MaxN> m_integral_controller;
  ect::DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN> m_filter;
  ect::DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN> m_target_filter;

  ect::MatrixN<N,MaxN> m_antiwindup_gain;
  ect::Value<N,MaxN> m_pos_deadband;
  ect::Value<N,MaxN> m_vel_cmd;
  ect::Value<N,MaxN> m_eff_cmd;
  ect::Value<N,MaxN> m_antiwindup;
  ect::Value<N,MaxN> m_max_effort;
  ect::Value<N,MaxN> m_target_vel;
  ect::Value<N,MaxN> m_target_eff;

  bool m_configured;
  bool m_use_target_torque;

  void callback(const sensor_msgs::JointStateConstPtr &msg);
  bool extractJoint(const sensor_msgs::JointState& msg, const std::vector<std::string>& name,
                      ect::Value<N,MaxN>& vel, ect::Value<N,MaxN>& eff);
};

using VelocityToTorqueController  = VelocityToTorqueControllerN<-1, cnr::control::max_num_axes>;
using VelocityToTorqueController1 = VelocityToTorqueControllerN<1>;
using VelocityToTorqueController3 = VelocityToTorqueControllerN<3>;
using VelocityToTorqueController6 = VelocityToTorqueControllerN<6>;
using VelocityToTorqueController7 = VelocityToTorqueControllerN<7>;

} // namespace control
} // namespace cnr


#include <cnr_velocity_to_torque_controller/internal/cnr_velocity_to_torque_controller_impl.h>



# endif  // cnr_vel_to_torque_control__20188101642
