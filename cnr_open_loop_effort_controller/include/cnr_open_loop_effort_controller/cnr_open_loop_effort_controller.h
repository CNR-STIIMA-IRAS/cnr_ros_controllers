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

class OpenLoopEffortController :
    public cnr_controller_interface::JointCommandController<hardware_interface::EffortJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  Eigen::VectorXd m_eff_cmd;
  Eigen::VectorXd m_max_effort;
  std::string     m_setpoint_topic_name;
  bool            m_configured;

  void callback(const boost::shared_ptr<const sensor_msgs::JointState> & msg);
  bool extractJoint(const sensor_msgs::JointState& msg);

  const std::string SP_TOPIC_ID = "sp";

};


}  // namespace control
}  // namespace cnr

#endif  // CNR_OPEN_LOOP_EFFORT_CONTROLLER__CNR_OPEN_LOOP_EFFORT_CONTROLLER__H
