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

template<int N, int MaxN=N>
class JointImpedanceControllerN :
    public JointCommandController<N, MaxN,
        hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:

  using Value = typename std::conditional<N==1, double, Eigen::Matrix<double,N,1,Eigen::ColMajor,MaxN> >::type;

  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);

protected:

  bool m_is_configured;
  
  typename JointRegulatorReference<N,MaxN>::Ptr m_regulator_input;
  typename JointRegulatorControlCommand<N,MaxN>::Ptr m_regulator_output;
  ImpedanceRegulatorN<N,MaxN> m_regulator;
  
  bool m_target_ok;
  bool m_effort_ok;
  bool m_use_wrench;
  
  typename ImpedanceRegulatorState<N,MaxN>::Ptr m_state0;

  Value m_q_target;
  Value m_qd_target;

  Value m_effort_db;
  Value m_effort;

  Eigen::Matrix<double,6,1> m_wrench_db;
  Value                     m_wrench_of_t_in_b;

  std::string   m_sensor_link;
  rosdyn::Chain m_chain_bs;

  void setTargetCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg);
  void setEffortCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg);
  void setWrenchCallback(const boost::shared_ptr<geometry_msgs::WrenchStamped const>& msg);

  ~JointImpedanceControllerN();
};

using JointImpedanceController  = JointImpedanceControllerN<-1, cnr::control::max_num_axes>;
using JointImpedanceController1 = JointImpedanceControllerN<1>;
using JointImpedanceController3 = JointImpedanceControllerN<3>;
using JointImpedanceController6 = JointImpedanceControllerN<6>;
using JointImpedanceController7 = JointImpedanceControllerN<7>;

}
}

#include <cnr_joint_impedance_controller/internal/cnr_joint_impedance_controller_impl.h>


#endif
