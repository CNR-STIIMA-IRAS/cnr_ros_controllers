#ifndef cnr_vel_to_torque_control__20188101642
#define cnr_vel_to_torque_control__20188101642

# include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
# include <hardware_interface/joint_command_interface.h>
# include <thread>
# include <mutex>
# include <boost/graph/graph_concepts.hpp>
# include <ros/ros.h>
# include <sensor_msgs/JointState.h>
# include <pluginlib/class_list_macros.h>

# include <subscription_notifier/subscription_notifier.h>
# include <eigen_state_space_systems/eigen_state_space_systems.h>
# include <eigen_state_space_systems/eigen_controllers.h>
# include <ros/callback_queue.h>


namespace cnr
{
namespace control
{

class VelocityToTorqueController :
    public cnr_controller_interface::JointCommandController<hardware_interface::EffortJointInterface>
{
public:
  bool doInit( );
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  bool m_well_init;
  bool m_use_feedback;

  eigen_control_toolbox::Controller m_controller;
  eigen_control_toolbox::Controller m_integral_controller;
  eigen_control_toolbox::DiscreteStateSpace m_filter;
  eigen_control_toolbox::DiscreteStateSpace m_target_filter;

  double m_pos_deadband;
  double m_antiwindup_gain;
  double m_vel_cmd;
  double m_eff_cmd;
  double m_antiwindup;
  double m_max_effort;

  double m_target_vel;
  double m_target_eff;

  bool m_configured;
  bool m_use_target_torque;

  void callback(const sensor_msgs::JointStateConstPtr &msg);
  bool extractJoint(const sensor_msgs::JointState msg, const std::string name, double& vel, double& eff);

};


} // namespace control
} // namespace cnr

# endif  // cnr_vel_to_torque_control__20188101642
