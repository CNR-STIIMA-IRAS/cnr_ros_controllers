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
template<int N,int MaxN, class H, class T>
class PositionToVelocityControllerBaseN:
    public cnr::control::JointCommandController<N,MaxN,H,T>
{
public:
  virtual bool doInit();
  virtual bool doStarting(const ros::Time& time);
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
  virtual bool doStopping(const ros::Time& time);

protected:
  cnr::control::PositionToVelocityControllerMathN<N,MaxN> ctrl;

  //! Position Target: it may be a double, if N==1, or a Matrix<double, N, 1> if N!=1
  ect::Value<N,MaxN> m_target_pos;
  //! Velocity Target: it may be a double, if N==1, or a Matrix<double, N, 1> if N!=1
  ect::Value<N,MaxN> m_target_vel;
  //! Effort Target: it may be a double, if N==1, or a Matrix<double, N, 1> if N!=1
  ect::Value<N,MaxN> m_target_eff;
  bool               m_configured;
  double             m_last_sp_time;

  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState msg, const std::vector<std::string>& name,
     typename ect::Value<N,MaxN>& pos, typename ect::Value<N,MaxN>& vel, typename ect::Value<N,MaxN>& eff);
};

//! alias, to make simpler
template<int N, int MaxN=N>
using PositionToVelocityControllerN = PositionToVelocityControllerBaseN<N, MaxN,
                                          hardware_interface::JointHandle,hardware_interface::VelocityJointInterface>;

//! List of the objects that are instatiated and located in the plugin
using PositionToVelocityController  = PositionToVelocityControllerN<-1, cnr::control::max_num_axes>;
using PositionToVelocityController1 = PositionToVelocityControllerN<1>;
using PositionToVelocityController3 = PositionToVelocityControllerN<3>;
using PositionToVelocityController6 = PositionToVelocityControllerN<6>;
using PositionToVelocityController7 = PositionToVelocityControllerN<7>;

template<int N, int MaxN=N>
using PositionToVelocityControllerFfwBaseN = PositionToVelocityControllerBaseN<N, MaxN,
                                 hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>;

template<int N, int MaxN=N>
class PositionToVelocityControllerFfwN : public PositionToVelocityControllerFfwBaseN<N, MaxN>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

};

using PositionToVelocityControllerFfw  = PositionToVelocityControllerFfwN<-1, cnr::control::max_num_axes>;
using PositionToVelocityControllerFfw1 = PositionToVelocityControllerFfwN<1>;
using PositionToVelocityControllerFfw3 = PositionToVelocityControllerFfwN<3>;
using PositionToVelocityControllerFfw6 = PositionToVelocityControllerFfwN<6>;
using PositionToVelocityControllerFfw7 = PositionToVelocityControllerFfwN<7>;

}  // namespace control
}  // namespace cnr

#include <cnr_position_to_velocity_controller/internal/cnr_position_to_velocity_controller_impl.h>

#endif  // CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER__H
