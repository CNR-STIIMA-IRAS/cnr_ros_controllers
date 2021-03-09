#ifndef CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_MATH_H
#define CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_MATH_H

#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <state_space_controllers/controllers.h>
#include <rosdyn_core/primitives.h>
#include <sensor_msgs/JointState.h>

namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{

typedef ect::Controller<-1,rosdyn::max_num_axes> ControllerX;
using DiscreteStateSpaceX = ect::DiscreteStateSpace<-1,-1,-1,
                                  rosdyn::max_num_axes,rosdyn::max_num_axes,rosdyn::max_num_axes>;


class PositionToVelocityControllerMath
{
public:
  PositionToVelocityControllerMath() = default;
  bool init(ros::NodeHandle& ctrl_nh, const rosdyn::VectorXd& speed_limit, std::string& what);
  bool update(const ros::Time& time,
              const rosdyn::VectorXd* const trg_pos,
              const rosdyn::VectorXd* const trg_vel,
              const rosdyn::VectorXd* const trg_eff,
              const double* const last_sp_time,
              const rosdyn::VectorXd& fb_pos,
              const rosdyn::VectorXd& fb_vel);
  bool starting(const rosdyn::VectorXd& fb_pos, const rosdyn::VectorXd& fb_vel, std::string& what);
  void stopping();

  const rosdyn::VectorXd& getPosCmd() const
  {
    return m_pos_cmd;
  }
  const rosdyn::VectorXd& getVelCmd() const
  {
    return m_vel_cmd;
  }
  const rosdyn::VectorXd& getEffCmd() const
  {
    return m_eff_cmd;
  }

protected:

  ControllerX         m_controller;
  ControllerX         m_integral_controller;
  DiscreteStateSpaceX m_pos_filter;
  DiscreteStateSpaceX m_target_pos_filter;

  bool m_use_feedback;
  bool m_interpolate_setpoint;
  double m_maximum_interpolation_time;
  rosdyn::VectorXd m_last_target_pos;
  rosdyn::VectorXd m_pos_minimum_error;
  rosdyn::VectorXd m_pos_maximum_error;
  rosdyn::VectorXd m_pos_deadband;
  rosdyn::MatrixXd m_antiwindup_gain;
  rosdyn::VectorXd m_pos_cmd;
  rosdyn::VectorXd m_vel_cmd;
  rosdyn::VectorXd m_eff_cmd;
  rosdyn::VectorXd m_antiwindup;
  rosdyn::VectorXd m_max_velocity;

  bool m_use_target_torque;
  bool m_use_target_velocity;
};


}
}

#include <cnr_position_to_velocity_controller/internal/cnr_position_to_velocity_math_impl.h>

#endif // CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_MATH_H
