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
#include <state_space_controllers/controllers.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{

typedef ect::Controller<-1,rosdyn::max_num_axes> ControllerX;
using DiscreteStateSpaceX = ect::DiscreteStateSpace<-1,-1,-1,
                                  rosdyn::max_num_axes,rosdyn::max_num_axes,rosdyn::max_num_axes>;


//!
class VelocityToTorqueController : public
    cnr::control::JointCommandController< hardware_interface::JointHandle, hardware_interface::EffortJointInterface>
{
public:
  bool doInit( );
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  bool m_well_init;
  bool m_use_feedback;

  ControllerX m_controller;
  ControllerX m_integral_controller;
  DiscreteStateSpaceX m_filter;
  DiscreteStateSpaceX m_target_filter;

  rosdyn::MatrixXd m_antiwindup_gain;
  rosdyn::VectorXd m_pos_deadband;
  rosdyn::VectorXd m_vel_cmd;
  rosdyn::VectorXd m_eff_cmd;
  rosdyn::VectorXd m_antiwindup;
  rosdyn::VectorXd m_max_effort;
  rosdyn::VectorXd m_target_vel;
  rosdyn::VectorXd m_target_eff;

  bool m_configured;
  bool m_use_target_torque;

  void callback(const sensor_msgs::JointStateConstPtr &msg);
  bool extractJoint(const sensor_msgs::JointState& msg, const std::vector<std::string>& name,
                      rosdyn::VectorXd& vel, rosdyn::VectorXd& eff);
};

} // namespace control
} // namespace cnr

# endif  // cnr_vel_to_torque_control__20188101642
