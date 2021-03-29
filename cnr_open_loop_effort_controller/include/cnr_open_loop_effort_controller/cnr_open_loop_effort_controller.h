#ifndef CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER__H
#define CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER__H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>


namespace cnr
{
namespace control
{

//!
class OpenLoopEffortController :
  public cnr::control::JointCommandController<hardware_interface::JointHandle,hardware_interface::EffortJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  rosdyn::VectorXd  m_eff_cmd;
  rosdyn::VectorXd  m_max_effort;

  std::string m_setpoint_topic_name;
  bool        m_configured;

  void callback(const boost::shared_ptr<const sensor_msgs::JointState> & msg);
  bool extractJoint(const sensor_msgs::JointState& msg);

  const std::string SP_TOPIC_ID = "sp";

};

}  // namespace control
}  // namespace cnr

#include <cnr_open_loop_effort_controller/internal/cnr_open_loop_effort_controller_impl.h>

#endif  // CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER__H
