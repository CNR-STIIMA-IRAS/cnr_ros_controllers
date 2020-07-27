#ifndef __cnr_pos_to_vel_control_math__
#define __cnr_pos_to_vel_control_math__

#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <eigen_state_space_systems/eigen_controllers.h>
#include <thread>
#include <mutex>
#include <boost/graph/graph_concepts.hpp>
#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

namespace cnr
{
namespace control
{

class PositionToVelocityControllerMath
{
public:

  PositionToVelocityControllerMath() {}
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period, const double& fb_pos, const double& fb_vel);
  void starting(const ros::Time& time, const double& fb_pos, const double& fb_vel);
  void stopping(const ros::Time& time);


  double&            getPosCmd()
  {
    return m_pos_cmd;
  }
  double&            getVelCmd()
  {
    return m_vel_cmd;
  }
  double&            getEffCmd()
  {
    return m_eff_cmd;
  }
  const std::string& getJointName() const
  {
    return m_joint_name;
  }
  const bool&        isWellInit() const
  {
    return m_well_init;
  }

protected:

  bool m_well_init;
  bool m_use_feedback;
  bool m_interpolate_setpoint;
  double m_maximum_interpolation_time;
  double m_last_sp_time;
  ros::CallbackQueue m_queue;
  boost::shared_ptr<ros::AsyncSpinner> m_spinner;

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
  double m_target_pos;
  double m_target_vel;
  double m_target_eff;

  double m_antiwindup;
  double m_max_velocity;


  std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> m_target_js_rec;

  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  bool m_configured;
  bool m_use_target_torque;
  bool m_use_target_velocity;
  bool m_error;
  double m_position_maximum_error;


  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState msg, const std::string name, double& pos, double& vel, double& eff);
  void stopThreads();
};


}
}

# endif
