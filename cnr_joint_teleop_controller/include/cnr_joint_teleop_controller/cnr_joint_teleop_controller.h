#ifndef cnr_joint_teleop_controller__20188101642
#define cnr_joint_teleop_controller__20188101642

#include <ros/ros.h>

#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>

#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <eigen_state_space_systems/eigen_controllers.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <name_sorting/name_sorting.h>
// #include <joint_teleop_gui/joint_teleop_gui.h>


namespace cnr
{
namespace control
{
class JointTeleopController: public cnr_controller_interface::JointController<hardware_interface::PosVelEffJointInterface>
{

public:
  JointTeleopController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

  void callback(const sensor_msgs::JointStateConstPtr msg);
  void delta();

protected:
  std::vector<hardware_interface::PosVelEffJointHandle> m_jh;

  std::vector<double> m_last_target_vel;
  std::vector<double> m_target_vel;
  std::vector<double> m_target_pos;
  std::vector<double> m_cmd_pos;
  std::vector<double> m_err_pos_delta;
  double m_err_delta;
  std::vector<double> m_saturated_vel;


  double m_time;
  bool m_configured;
  bool m_compared;
  bool m_check1 = false;
  bool m_check2 = false;

  double m_err;
  double m_err_old = 0;

};
}
}







#endif
