#pragma once // workaraound qtcreator clang-tidy

#ifndef CNR_POSITION_TO_VELOCITY_CNR__POSITION_TO_VELOCITY_MATH_IMPL_H
#define CNR_POSITION_TO_VELOCITY_CNR__POSITION_TO_VELOCITY_MATH_IMPL_H

#include <ros/node_handle.h>
#include <Eigen/Core>
#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_math.h>
#include <urdf/model.h>

namespace cnr
{
namespace control
{

template<int N,int MaxN>
inline bool PositionToVelocityControllerMathN<N,MaxN>::init(ros::NodeHandle& nh,
      const typename ect::Value<N,MaxN>& speed_limit)
{
  m_use_target_velocity = false;
  if (!nh.getParam("use_target_velocity", m_use_target_velocity))
  {
    ROS_DEBUG_STREAM(nh.getNamespace() + "/use_target_velocity does not exist, set FALSE");
  }

  m_use_target_torque = false;
  if (!nh.getParam("use_target_torque", m_use_target_torque))
  {
    ROS_DEBUG_STREAM(nh.getNamespace() + "/use_target_torque does not exist, set FALSE");
  }

  m_target_pos_filter.importMatricesFromParam(nh, "target_pos_filter");
  m_pos_filter.importMatricesFromParam(nh, "pos_filter");

  std::string what;
  if(m_controller.importMatricesFromParam(nh, "controller",what)==-1)
  {
    ROS_ERROR_STREAM(what);
    return false;
  }
  if(m_integral_controller.importMatricesFromParam(nh, "integral_controller",what)==-1)
  {
    ROS_ERROR_STREAM(what);
    return false;
  }

  if(!eigen_utils::resize(m_pos_minimum_error, eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_pos_maximum_error, eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_last_target_pos  , eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_pos_deadband     , eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_antiwindup_gain  , eigen_utils::size(speed_limit), eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_pos_cmd          , eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_vel_cmd          , eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_eff_cmd          , eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_antiwindup       , eigen_utils::size(speed_limit))
  || !eigen_utils::resize(m_max_velocity     , eigen_utils::size(speed_limit)))
  {
    return false;
  }

  eigen_utils::setZero(m_pos_minimum_error);
  eigen_utils::setConstant(m_pos_maximum_error,0.1);
  eigen_utils::setZero(m_last_target_pos       );
  eigen_utils::setZero(m_pos_deadband          );
  eigen_utils::setDiagonal(m_antiwindup_gain, 1.0);
  eigen_utils::setZero(m_pos_cmd               );
  eigen_utils::setZero(m_vel_cmd               );
  eigen_utils::setZero(m_eff_cmd               );
  eigen_utils::setZero(m_antiwindup            );
  eigen_utils::setZero(m_max_velocity          );

  if(!rosparam_utilities::getParam(nh,"position_minimum_error", m_pos_minimum_error, what, &m_pos_minimum_error))
  {
    ROS_WARN("%s", what.c_str());
  }

  if(!rosparam_utilities::getParam(nh,"position_maximum_error", m_pos_maximum_error, what, &m_pos_maximum_error))
  {
    ROS_WARN("%s", what.c_str());
  }

  if (!nh.getParam("interpolate_setpoint", m_interpolate_setpoint))
  {
    ROS_WARN("interpolate_setpoint specified, set false");
    m_interpolate_setpoint = false;
  }
  else if (!nh.getParam("maximum_interpolation_time", m_maximum_interpolation_time))
  {
    ROS_WARN("maximum_interpolation_time specified, set 10 ms");
    m_maximum_interpolation_time = 0.01;
  }

  if(!rosparam_utilities::getParam(nh,"antiwindup_ratio", m_antiwindup_gain, what, &m_antiwindup_gain))
  {
    ROS_WARN("%s", what.c_str());
  }

  m_max_velocity = speed_limit;
  return true;

}

template<int N,int MaxN>
inline void PositionToVelocityControllerMathN<N,MaxN>::starting(const ect::Value<N,MaxN>& fb_pos,
    const ect::Value<N,MaxN>& fb_vel)
{
  ect::Value<N,MaxN> init_pos;
  init_pos = fb_pos;
  m_last_target_pos = fb_pos;
  m_pos_filter.setStateFromLastIO(init_pos, init_pos);

  init_pos = fb_pos;
  m_target_pos_filter.setStateFromLastIO(init_pos, init_pos);

  ect::Value<N,MaxN> init_vel;
  if (m_use_target_velocity)
  {
    init_vel = fb_vel - fb_vel;
  }
  else
  {
    init_vel = fb_vel;
  }
  ect::Value<N,MaxN> init_error=m_target_pos_filter.y() - m_pos_filter.y();

  m_controller.setStateFromLastIO(init_error, init_vel);
  m_integral_controller.setStateFromLastIO(init_error, init_vel); // TODO fix INITIALIZATION of two controllers
  eigen_utils::setZero(m_antiwindup);
}

template<int N,int MaxN>
inline void PositionToVelocityControllerMathN<N,MaxN>::stopping()
{
  eigen_utils::setZero(m_vel_cmd);
  eigen_utils::setZero(m_eff_cmd);
}

template<int N,int MaxN>
inline bool PositionToVelocityControllerMathN<N,MaxN>::update(const ros::Time& time,
      const ect::Value<N,MaxN>* const trg_pos,
      const ect::Value<N,MaxN>* const trg_vel,
      const ect::Value<N,MaxN>* const trg_eff,
      const double* const last_sp_time,
      const ect::Value<N,MaxN>& fb_pos,
      const ect::Value<N,MaxN>& /*fb_vel*/)
{
  try
  {
    ect::Value<N,MaxN> zero_val;
    eigen_utils::resize(zero_val, eigen_utils::rows(*trg_pos));
    eigen_utils::setZero(zero_val);

    ect::Value<N,MaxN> filter_output             = zero_val;
    ect::Value<N,MaxN> target_filter_output      = zero_val;
    ect::Value<N,MaxN> controller_input          = zero_val;
    ect::Value<N,MaxN> controller_output         = zero_val;
    ect::Value<N,MaxN> integral_controller_input = zero_val;
    ect::Value<N,MaxN> integral_controller_output= zero_val;

    filter_output = m_pos_filter.update(fb_pos);

    if(trg_pos)
    {
      ect::Value<N,MaxN> target_pos = (*trg_pos);
      if (m_interpolate_setpoint && ((time.toSec() - *last_sp_time) <= m_maximum_interpolation_time))
      {
        target_pos = m_last_target_pos + (*trg_vel) * (time.toSec() - *last_sp_time);
      }
      target_filter_output = m_target_pos_filter.update(target_pos);
      m_last_target_pos = target_filter_output;
    }
    else
    {
      target_filter_output = m_target_pos_filter.update(m_target_pos_filter.y());
    }

    controller_input = target_filter_output - filter_output; //controller error

    if(eigen_utils::norm(controller_input) > eigen_utils::norm(m_pos_maximum_error))
    {
      ROS_ERROR("Exceeded the position_maximum_error!");
      eigen_utils::setZero(m_vel_cmd);
      return false;
    }
    integral_controller_input = target_filter_output - filter_output + m_antiwindup_gain * m_antiwindup; //integral controller error

    controller_output = m_controller.update(controller_input);
    integral_controller_output = m_integral_controller.update(integral_controller_input);

    m_pos_cmd = target_filter_output;
    m_vel_cmd = controller_output + integral_controller_output + ( m_use_target_velocity && trg_vel ? *trg_vel : zero_val );
    m_eff_cmd = m_use_target_torque && trg_eff ? *trg_eff : zero_val;

    for(int i=0; i<eigen_utils::size(m_pos_cmd);i++)
    {
      if(eigen_utils::at(m_vel_cmd,i) > eigen_utils::at(m_max_velocity,i))
      {
        eigen_utils::at(m_antiwindup,i) = eigen_utils::at(m_max_velocity,i) - eigen_utils::at(m_vel_cmd,i);
        eigen_utils::at(m_vel_cmd,i) = eigen_utils::at(m_max_velocity,i);
      }
      else if (eigen_utils::at(m_vel_cmd,i) < -eigen_utils::at(m_max_velocity,i))
      {
        eigen_utils::at(m_antiwindup,i) = -eigen_utils::at(m_max_velocity,i) - eigen_utils::at(m_vel_cmd,i);
        eigen_utils::at(m_vel_cmd,i) = -eigen_utils::at(m_max_velocity,i);
      }
      else
      {
        eigen_utils::at(m_antiwindup,i) = 0;
      }
    }

  }
  catch (...)
  {
    eigen_utils::setZero(m_vel_cmd);
    return false;
  }
  return true;
}

}
}
#endif  // CNR_POSITION_TO_VELOCITY_CNR__POSITION_TO_VELOCITY_MATH_IMPL_H
