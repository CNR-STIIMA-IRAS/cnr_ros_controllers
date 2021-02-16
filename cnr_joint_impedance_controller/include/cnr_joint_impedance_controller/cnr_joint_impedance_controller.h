#ifndef cnr_joint_impedance_controller__cnr_joint_impedance_controller_h
#define cnr_joint_impedance_controller__cnr_joint_impedance_controller_h

#include <thread>
#include <mutex>
#include <Eigen/Core>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <rosdyn_core/primitives.h>

#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator.h>

namespace cnr
{
namespace control
{

class JointImpedanceController :
    public JointCommandController<hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);

protected:

  bool m_is_configured;
  rosdyn::Link m_root_link;  //link primitivo da cui parte la catena cinematica (world ad esempio)

  
  typename JointRegulatorReference::Ptr m_regulator_input;
  typename JointRegulatorControlCommand::Ptr m_regulator_output;
  ImpedanceRegulator m_regulator;
  
  bool m_target_ok;
  bool m_effort_ok;
  bool m_use_wrench;
  
  typename ImpedanceRegulatorState::Ptr m_state0;

  rosdyn::VectorXd m_q_target;
  rosdyn::VectorXd m_qd_target;

  rosdyn::VectorXd m_effort_db;
  rosdyn::VectorXd m_effort;

  Eigen::Vector6d m_wrench_db;
  rosdyn::VectorXd          m_wrench_of_t_in_b;

  std::string   m_sensor_link;
  rosdyn::Chain m_chain_bs;

  void setTargetCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg);
  void setEffortCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg);
  void setWrenchCallback(const boost::shared_ptr<geometry_msgs::WrenchStamped const>& msg);

  ~JointImpedanceController();
};

}
}

#include <cnr_joint_impedance_controller/internal/cnr_joint_impedance_controller_impl.h>


#endif
