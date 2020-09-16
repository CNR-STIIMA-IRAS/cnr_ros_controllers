#ifndef cnr_joint_impedance_sensor__20190123
#define cnr_joint_impedance_sensor__20190123

#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <subscription_notifier/subscription_notifier.h>
#include <rosdyn_core/primitives.h>
#include <thread>
#include <mutex>
#include <boost/graph/graph_concepts.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/callback_queue.h>
#include <name_sorting/name_sorting.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace cnr
{
namespace control
{

class JointImpedanceController :
    public cnr_controller_interface::JointCommandController<hardware_interface::PosVelEffJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);

protected:

  bool m_is_configured;
  bool m_target_ok;
  bool m_effort_ok;
  bool m_use_wrench;

  Eigen::VectorXd m_jtarget;
  Eigen::VectorXd m_jDtarget;

  Eigen::VectorXd m_x0;
  Eigen::VectorXd m_Dx0;
  Eigen::VectorXd m_x;
  Eigen::VectorXd m_Dx;
  Eigen::VectorXd m_DDx;
  Eigen::VectorXd m_Jinv;
  Eigen::VectorXd m_damping;
  Eigen::VectorXd m_damping_dafault;
  Eigen::VectorXd m_k;
  Eigen::VectorXd m_k_default;
  Eigen::VectorXd m_k_new;
  Eigen::VectorXd m_torque_deadband;
  Eigen::Matrix<double,6,1> m_wrench_deadband;

  Eigen::VectorXd m_torque;
  Eigen::VectorXd m_wrench_of_t_in_b;

  std::string m_sensor_link;

  rosdyn::ChainPtr m_chain_bs;

  void setTargetCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg);
  void setEffortCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg);
  void setWrenchCallback(const boost::shared_ptr<geometry_msgs::WrenchStamped const>& msg);

  ~JointImpedanceController();

};


}
}


#endif
