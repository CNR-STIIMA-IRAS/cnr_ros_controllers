#pragma once // workaraound qtcreator clang-tidy

#ifndef CNR_POSITION_TO_VELOCITY_CNR__POSITION_TO_VELOCITY_MATH_IMPL_H
#define CNR_POSITION_TO_VELOCITY_CNR__POSITION_TO_VELOCITY_MATH_IMPL_H

#include <ros/node_handle.h>
#include <Eigen/Core>
#include <rosparam_utilities/rosparam_utilities.h>
#include <state_space_ros/ros_params.h>
#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_math.h>
//#include <urdf/model.h>

namespace ect = eigen_control_toolbox;
namespace eu  = eigen_utils;
namespace ru  = rosparam_utilities;

namespace cnr
{
namespace control
{

bool append_string(std::string& what, bool ok, const std::string& msg)
{
  std::string ch   =  ok ? "[?]" : "[!]";
  std::string msg_ = msg.size()>0 ? ch + " " + msg : "";

  what += msg_.size()>0 ? 
	  ( (what.size()>0 ? "\n" : "") + msg_ ) : "";
  return ok;
}

//!
inline bool PositionToVelocityControllerMath::init(
    ros::NodeHandle& nh,const rosdyn::VectorXd& speed_limit,std::string& what)
{
  what = "";

  //======================= CHECK IF ELEMENT ARE RESIZABLE (STATIC vs DYN ALLOCATION, and check malloc)
  if(!eu::resize(m_pos_minimum_error, eu::size(speed_limit))
  || !eu::resize(m_pos_maximum_error, eu::size(speed_limit))
  || !eu::resize(m_last_target_pos  , eu::size(speed_limit))
  || !eu::resize(m_pos_deadband     , eu::size(speed_limit))
  || !eu::resize(m_antiwindup_gain  , eu::size(speed_limit), eu::size(speed_limit))
  || !eu::resize(m_pos_cmd          , eu::size(speed_limit))
  || !eu::resize(m_vel_cmd          , eu::size(speed_limit))
  || !eu::resize(m_eff_cmd          , eu::size(speed_limit))
  || !eu::resize(m_antiwindup       , eu::size(speed_limit))
  || !eu::resize(m_max_velocity     , eu::size(speed_limit)))
  {
    what += (what.size()>0? "\n[!] " : "[!] ") + std::string("Error in resize a vector");
    std::cout << what << std::endl;
    return false;
  }
  //=================================

  //================================= default values
  eu::setZero(m_pos_minimum_error);
  eu::setConstant(m_pos_maximum_error,0.1);
  eu::setZero(m_last_target_pos       );
  eu::setZero(m_pos_deadband          );
  eu::setDiagonal(m_antiwindup_gain, 1.0);
  eu::setZero(m_pos_cmd               );
  eu::setZero(m_vel_cmd               );
  eu::setZero(m_eff_cmd               );
  eu::setZero(m_antiwindup            );
  eu::setZero(m_max_velocity          );
  m_interpolate_setpoint = false;
  m_maximum_interpolation_time = 0.01;
  m_use_target_velocity = false;
  m_use_target_torque = false;
  //================================= default values

  // PARAM
  bool ok = true;
  std::string msg;
  std::vector<double> pos_minimum_error, pos_maximum_error,  antiwindup_gain;
  ok &= append_string( what,ru::getParam(nh,"position_minimum_error"    , pos_minimum_error           , msg, &pos_minimum_error   ), msg);
  ok &= append_string( what,ru::getParam(nh,"position_maximum_error"    , pos_maximum_error           , msg, &pos_maximum_error   ), msg);
  ok &= append_string( what,ru::getParam(nh,"interpolate_setpoint"      , m_interpolate_setpoint      , msg, &m_interpolate_setpoint), msg) ;
  ok &= append_string( what,ru::getParam(nh,"maximum_interpolation_time", m_maximum_interpolation_time, msg, &m_maximum_interpolation_time), msg);
  ok &= append_string( what,ru::getParam(nh,"antiwindup_ratio"          , antiwindup_gain             , msg, &antiwindup_gain), msg);
  ok &= append_string( what,ru::getParam(nh,"use_target_velocity"       , m_use_target_velocity       , msg, &m_use_target_velocity), msg);
  ok &= append_string( what,ru::getParam(nh,"use_target_torque"         , m_use_target_torque         , msg, &m_use_target_torque), msg );
  ok &= append_string( what,ect::setMatricesFromParam(m_target_pos_filter  ,nh, "target_pos_filter"   , msg), msg );
  ok &= append_string( what,ect::setMatricesFromParam(m_pos_filter         ,nh, "pos_filter"          , msg), msg );
  ok &= append_string( what,ect::setMatricesFromParam(m_controller         ,nh, "controller"          , msg), msg );
  ok &= append_string( what,ect::setMatricesFromParam(m_integral_controller,nh, "integral_controller" , msg), msg );

  //=============

  std::vector<std::pair<int, std::string>> checks2 =
  {
    { m_target_pos_filter.uDim() == eu::size(speed_limit), "Wrong Target Pos Filter input dimension"},
    { m_target_pos_filter.yDim() == eu::size(speed_limit), "Wrong Target Pos Filter output dimension"},
    { m_pos_filter.uDim() == eu::size(speed_limit) ? 1 : -1, "Wrong Pos Filter input dimension"},
    { m_pos_filter.yDim() == eu::size(speed_limit) ? 1 : -1, "Wrong Pos Filter output dimension"},
    { m_controller.uDim() == eu::size(speed_limit) ? 1 : -1, "Wrong Controller input dimension"},
    { m_controller.yDim() == eu::size(speed_limit) ? 1 : -1, "Wrong Controller output dimension"},
    { m_integral_controller.uDim() == eu::size(speed_limit) ? 1 : -1, "Wrong Integral Controller input dimension"},
    { m_integral_controller.yDim() == eu::size(speed_limit) ? 1 : -1, "Wrong Integral Controller output dimension"},
    { eu::rows(pos_minimum_error) == eu::rows(m_pos_minimum_error) || eu::rows(pos_minimum_error)==1, "Wrong position_minimum_error dimension"},
    { eu::rows(pos_maximum_error) == eu::rows(m_pos_maximum_error) || eu::rows(pos_maximum_error)==1, "Wrong position_maximum_error dimension"},
    { eu::rows(antiwindup_gain)   == eu::rows(m_antiwindup_gain)   || eu::rows(antiwindup_gain)==1, "Wrong antiwindup_gain dimension, got" 
      + std::to_string(eu::rows(antiwindup_gain)) +", expected 1 or " + std::to_string(eu::rows(m_antiwindup_gain)) },
  };

  for(size_t i=0;i<checks2.size();i++)
  {
    ok &= checks2.at(i).first;
    std::string ch = (checks2.at(i).first == -1) ? "[!]" : "[?]";
    what += checks2.at(i).first==1 ? "" : ( (what.size()>0? "\n" : "" ) + ch + " " + checks2.at(i).second );
  }
  if(!ok)
  {
    return false;
  }

  if(eu::rows(pos_minimum_error) == eu::rows(m_pos_minimum_error))
  {
    eu::copy(m_pos_minimum_error, pos_minimum_error);
  }
  else
  {
    eu::setConstant(m_pos_minimum_error, pos_minimum_error.at(0));
  }

  if(eu::rows(pos_maximum_error) == eu::rows(m_pos_maximum_error))
  {
    eu::copy(m_pos_maximum_error, pos_maximum_error);
  }
  else
  {
    eu::setConstant(m_pos_maximum_error, pos_maximum_error.at(0));
  }

  if(eu::rows(antiwindup_gain) == eu::rows(m_antiwindup_gain))
  {
    eu::setDiagonal(m_antiwindup_gain, antiwindup_gain);
  }
  else
  {
    eu::setDiagonal(m_antiwindup_gain, antiwindup_gain.at(0));
  }

  m_max_velocity = speed_limit;

  return true;

}

//!
inline bool PositionToVelocityControllerMath::starting(const rosdyn::VectorXd& fb_pos, const rosdyn::VectorXd& fb_vel, std::string& what)
{
  if(fb_pos.rows()!=m_last_target_pos.rows() || fb_vel.rows()!=m_last_target_pos.rows())
  {
    what = "The input args has incorrect dimension. fb pos: " + std::to_string(fb_pos.rows())+", fb vel: "+std::to_string(fb_vel.rows());
    what += " last pos: (" + std::to_string(m_last_target_pos.rows())+"x"+std::to_string(m_last_target_pos.cols())+")";
    return false;
  }
  if(m_pos_filter.uDim()!=fb_pos.rows() || m_target_pos_filter.uDim()!=fb_pos.rows())
  {
    what = "The input args has incorrect dimension. fb pos: " + std::to_string(fb_pos.rows())+", fb vel: "+std::to_string(fb_vel.rows());
    what += " pos filter: " + std::to_string(m_pos_filter.uDim()) + "  target pos filter: "+std::to_string( m_target_pos_filter.uDim())+")";
    return false;
  }
  if(m_pos_filter.yDim()!=fb_pos.rows() || m_target_pos_filter.yDim()!=fb_pos.rows())
  {
    what = "The input args has incorrect dimension. fb pos: " + std::to_string(fb_pos.rows())+", fb vel: "+std::to_string(fb_vel.rows());
    what += " pos filter: " + std::to_string(m_pos_filter.yDim()) + "  target pos filter: "+std::to_string( m_target_pos_filter.yDim())+")";
    return false;
  }
  
  rosdyn::VectorXd init_pos;
  init_pos = fb_pos;
  m_last_target_pos = fb_pos;
  m_pos_filter.setStateFromLastInput(init_pos);

  init_pos = fb_pos;
  m_target_pos_filter.setStateFromLastInput(init_pos);

  rosdyn::VectorXd init_vel;
  if (m_use_target_velocity)
  {
    init_vel = fb_vel - fb_vel;
  }
  else
  {
    init_vel = fb_vel;
  }
  rosdyn::VectorXd init_error=m_target_pos_filter.y() - m_pos_filter.y();

  m_controller.setStateFromLastIO(init_error, init_vel);
  m_integral_controller.setStateFromLastIO(init_error, init_vel); // TODO fix INITIALIZATION of two controllers
  eu::setZero(m_antiwindup);
  return true;
}

//!
inline void PositionToVelocityControllerMath::stopping()
{
  eu::setZero(m_vel_cmd);
  eu::setZero(m_eff_cmd);
}

//!
inline bool PositionToVelocityControllerMath::update(const ros::Time& time,
      const rosdyn::VectorXd* const trg_pos,
      const rosdyn::VectorXd* const trg_vel,
      const rosdyn::VectorXd* const trg_eff,
      const double* const last_sp_time,
      const rosdyn::VectorXd& fb_pos,
      const rosdyn::VectorXd& /*fb_vel*/)
{
  try
  {
    rosdyn::VectorXd zero_val;
    eu::resize(zero_val, eu::rows(fb_pos));
    eu::setZero(zero_val);

    rosdyn::VectorXd filter_output             = zero_val;
    rosdyn::VectorXd target_filter_output      = zero_val;
    rosdyn::VectorXd controller_input          = zero_val;
    rosdyn::VectorXd controller_output         = zero_val;
    rosdyn::VectorXd integral_controller_input = zero_val;
    rosdyn::VectorXd integral_controller_output= zero_val;

    filter_output = m_pos_filter.update(fb_pos);

    if(trg_pos)
    {
      rosdyn::VectorXd target_pos = (*trg_pos);
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

    if(eu::norm(controller_input) > eu::norm(m_pos_maximum_error))
    {
      ROS_ERROR("Exceeded the position_maximum_error!");
      eu::setZero(m_vel_cmd);
      return false;
    }
    auto s = [](const std::string w, auto & v) { std::cout <<w << ": " << eu::rows(v) << "x" << eu::cols(v) << std::endl;};
       
    integral_controller_input = target_filter_output - filter_output + m_antiwindup_gain * m_antiwindup; //integral controller error

    controller_output = m_controller.update(controller_input);
    integral_controller_output = m_integral_controller.update(integral_controller_input);

    m_pos_cmd = target_filter_output;
    m_vel_cmd = controller_output + integral_controller_output + ( m_use_target_velocity && trg_vel ? *trg_vel : zero_val );
    m_eff_cmd = m_use_target_torque && trg_eff ? *trg_eff : zero_val;

    for(int i=0; i<eu::size(m_pos_cmd);i++)
    {
      if(eu::at(m_vel_cmd,i) > eu::at(m_max_velocity,i))
      {
        eu::at(m_antiwindup,i) = eu::at(m_max_velocity,i) - eu::at(m_vel_cmd,i);
        eu::at(m_vel_cmd,i) = eu::at(m_max_velocity,i);
      }
      else if (eu::at(m_vel_cmd,i) < -eu::at(m_max_velocity,i))
      {
        eu::at(m_antiwindup,i) = -eu::at(m_max_velocity,i) - eu::at(m_vel_cmd,i);
        eu::at(m_vel_cmd,i) = -eu::at(m_max_velocity,i);
      }
      else
      {
        eu::at(m_antiwindup,i) = 0;
      }
    }

  }
  catch (...)
  {
    eu::setZero(m_vel_cmd);
    return false;
  }
  return true;
}

}
}
#endif  // CNR_POSITION_TO_VELOCITY_CNR__POSITION_TO_VELOCITY_MATH_IMPL_H
