#ifndef CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER__H
#define CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER__H

#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvelacc_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_math.h>

//! alias
namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{

/**
 * @brief The template is designed to integrate a state-based simple controller
 * The template depends of the number of the axes, in order to fix the dimension of the object,
 * without the necessity of dynamic heap allocation in runtime
 * The setpoint(target) is super-imposed by a topic, i.e., the object subcribes a topic of type JoinState msg
 * The name of the topic is specified by the parameter:
 * /<robot_hardware_namespace>/<controller_namespace>/setpoint_topic_name
 */
template<class H, class T>
class PositionToVelocityControllerBase:
    public cnr::control::JointCommandController<H,T>
{
public:
  virtual bool doInit();
  virtual bool doStarting(const ros::Time& time);
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
  virtual bool doStopping(const ros::Time& time);

protected:
  cnr::control::PositionToVelocityControllerMath ctrl;

  rosdyn::VectorXd m_target_pos;
  rosdyn::VectorXd m_target_vel;
  rosdyn::VectorXd m_target_eff;
  bool             m_configured;
  double           m_last_sp_time;

  size_t           m_command_pub;
  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState msg, const std::vector<std::string>& name,
                        rosdyn::VectorXd& pos, rosdyn::VectorXd& vel, rosdyn::VectorXd& eff);
};

//! alias, to make simpler
using PositionToVelocityController = PositionToVelocityControllerBase<
                                          hardware_interface::JointHandle,hardware_interface::VelocityJointInterface>;


using PositionToVelocityControllerFfwBase = PositionToVelocityControllerBase<
                                 hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>;

class PositionToVelocityControllerFfw : public PositionToVelocityControllerFfwBase
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);
};

}  // namespace control
}  // namespace cnr

#endif  // CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER__H
