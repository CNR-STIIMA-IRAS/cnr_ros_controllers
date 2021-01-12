#ifndef CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_MATH_H
#define CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_MATH_H

#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <eigen_state_space_systems/eigen_controllers.h>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{

template<int N, int MaxN>
class PositionToVelocityControllerMathN
{
public:
  PositionToVelocityControllerMathN() = default;
  bool init(ros::NodeHandle& ctrl_nh, const typename ect::Value<N,MaxN>& speed_limit);
  bool update(const ros::Time& time,
              const ect::Value<N,MaxN>* const trg_pos,
              const ect::Value<N,MaxN>* const trg_vel,
              const ect::Value<N,MaxN>* const trg_eff,
              const double* const last_sp_time,
              const ect::Value<N,MaxN>& fb_pos,
              const ect::Value<N,MaxN>& fb_vel);
  void starting(const ect::Value<N,MaxN>& fb_pos, const ect::Value<N,MaxN>& fb_vel);
  void stopping();

  const ect::Value<N,MaxN>& getPosCmd() const
  {
    return m_pos_cmd;
  }
  const ect::Value<N,MaxN>& getVelCmd() const
  {
    return m_vel_cmd;
  }
  const ect::Value<N,MaxN>& getEffCmd() const
  {
    return m_eff_cmd;
  }

protected:

  ect::Controller<N,MaxN> m_controller;
  ect::Controller<N,MaxN> m_integral_controller;
  ect::DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN> m_pos_filter;
  ect::DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN> m_target_pos_filter;

  bool m_use_feedback;
  bool m_interpolate_setpoint;
  double m_maximum_interpolation_time;
  ect::Value<N,MaxN> m_last_target_pos;
  ect::Value<N,MaxN> m_pos_minimum_error;
  ect::Value<N,MaxN> m_pos_maximum_error;
  ect::Value<N,MaxN> m_pos_deadband;
  ect::MatrixN<N,MaxN> m_antiwindup_gain;
  ect::Value<N,MaxN> m_pos_cmd;
  ect::Value<N,MaxN> m_vel_cmd;
  ect::Value<N,MaxN> m_eff_cmd;
  ect::Value<N,MaxN> m_antiwindup;
  ect::Value<N,MaxN> m_max_velocity;

  bool m_use_target_torque;
  bool m_use_target_velocity;
};


}
}

#include <cnr_position_to_velocity_controller/internal/cnr_position_to_velocity_math_impl.h>

#endif // CNR_POSITION_TO_VELOCITY_CONTROLLER__CNR_POSITION_TO_VELOCITY_CONTROLLER_MATH_H
