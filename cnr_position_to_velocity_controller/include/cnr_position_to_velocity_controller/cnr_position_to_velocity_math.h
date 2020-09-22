#ifndef __cnr_pos_to_vel_control_math__
#define __cnr_pos_to_vel_control_math__

#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <eigen_state_space_systems/eigen_controllers.h>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace cnr
{
namespace control
{

class PositionToVelocityControllerMath
{
public:

  PositionToVelocityControllerMath() {}
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  bool update(const ros::Time& time, const ros::Duration& period, const double *trg_pos, const double *trg_vel, const double *trg_eff,
              const double * last_sp_time, const double& fb_pos, const double& fb_vel);
  void starting(const ros::Time& time, const double& fb_pos, const double& fb_vel);
  void stopping(const ros::Time& time);

  const double& getPosCmd() const
  {
    return m_pos_cmd;
  }
  const double& getVelCmd() const
  {
    return m_vel_cmd;
  }
  const double& getEffCmd() const
  {
    return m_eff_cmd;
  }
  const std::string& getJointName() const
  {
    return m_joint_name;
  }

protected:

  bool m_use_feedback;
  bool m_interpolate_setpoint;
  double m_maximum_interpolation_time;
  double m_last_target_pos;

  eigen_control_toolbox::Controller m_controller;
  eigen_control_toolbox::Controller m_integral_controller;
  eigen_control_toolbox::DiscreteStateSpace m_pos_filter;
  eigen_control_toolbox::DiscreteStateSpace m_target_pos_filter;

  double m_pos_deadband;
  std::string m_joint_name;
  double m_position_minimum_error;
  double m_antiwindup_gain;
  double m_pos_cmd;
  double m_vel_cmd;
  double m_eff_cmd;

  double m_antiwindup;
  double m_max_velocity;

  bool m_use_target_torque;
  bool m_use_target_velocity;
  double m_position_maximum_error;


  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState msg, const std::string name, double& pos, double& vel, double& eff);
  void stopThreads();
};


}
}

# endif
