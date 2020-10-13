#ifndef cnr_joint_impedance_sensor__20190123
#define cnr_joint_impedance_sensor__20190123

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
    public cnr_controller_interface::JointCommandController<hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);

protected:

  bool m_is_configured;
  
  cnr_impedance_regulator::ImpedanceRegulatorReferencePtr m_regulator_input;
  cnr_impedance_regulator::ImpedanceRegulatorControlCommandPtr m_regulator_output;
  cnr_impedance_regulator::ImpedanceRegulator m_regulator;
  
  bool m_target_ok;
  bool m_effort_ok;
  bool m_use_wrench;
  
  cnr_impedance_regulator::ImpedanceRegulatorStatePtr m_state0;

  Eigen::VectorXd m_q_target;
  Eigen::VectorXd m_qd_target;

  Eigen::VectorXd m_effort_db;
  Eigen::VectorXd m_effort;

  Eigen::Matrix<double,6,1> m_wrench_db;
  Eigen::VectorXd           m_wrench_of_t_in_b;

  std::string      m_sensor_link;
  rosdyn::ChainPtr m_chain_bs;

  void setTargetCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg);
  void setEffortCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg);
  void setWrenchCallback(const boost::shared_ptr<geometry_msgs::WrenchStamped const>& msg);

  ~JointImpedanceController();

};


}
}


#endif
