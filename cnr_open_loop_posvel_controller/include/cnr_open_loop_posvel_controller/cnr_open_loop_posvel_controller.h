#ifndef CNR_OPEN_LOOP_POSITION_CONTROLLER__CNR_OPEN_LOOP_POSITION_CONTROLLER__H
#define CNR_OPEN_LOOP_POSITION_CONTROLLER__CNR_OPEN_LOOP_POSITION_CONTROLLER__H


#include <hardware_interface/joint_command_interface.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

namespace cnr
{
namespace control
{

/**
 * @brief The OpenLoopPosVelController class
 */
class OpenLoopPosVelController :
  public cnr::control::JointCommandController<hardware_interface::PosVelEffJointHandle,
                                                            hardware_interface::PosVelEffJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  std::string m_setpoint_topic_name;
  bool        m_configured;

  void callback(const sensor_msgs::JointStateConstPtr& msg);
  bool extractJoint(const sensor_msgs::JointState& msg);

  const std::string   SP_TOPIC_ID = "sp";
};

}  // namespace control
}  // namespace cnr

# endif  // CNR_OPEN_LOOP_POSITION_CONTROLLER__CNR_OPEN_LOOP_POSITION_CONTROLLER__H
