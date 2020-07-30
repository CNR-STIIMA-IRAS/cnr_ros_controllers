#include <boost/algorithm/string/predicate.hpp>
#include <cnr_cartesian_motion_ctrl/cnr_cartesian_motion_ctrl.h>
#include <pluginlib/class_list_macros.h>

#include <locale>         // std::locale, std::tolower
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_core/spacevect_algebra.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <controller_interface/controller_base.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::CartMotionController, controller_interface::ControllerBase);

namespace std
{
inline std::string to_string( const std::vector<std::string>& vals )
{
  std::string ret = "< ";
  for( auto const & val : vals ) ret += val + ", ";
  ret += " >";
  return ret;
}

inline std::string to_string( const std::string& val )
{
  return val;
}

inline std::string to_string( const bool& val )
{
  return val ? "TRUE" : "FALSE";
}

}



#define GET_AND_RETURN( nh, param, value )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_ERROR("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    return false;\
  }



#define GET_AND_DEFAULT( nh, param, value, def )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_WARN("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    ROS_WARN("Default value '%s' superimposed. ", std::to_string( def ).c_str() );\
    value=def;\
  }


namespace cnr
{
namespace control
{


bool CartMotionController::doInit()
{
  CNR_TRACE_START(*m_logger);

  GET_AND_DEFAULT( getControllerNh(), "goal_linear_tollerance" , goal_linear_tollerance_, 1e-3);
  GET_AND_DEFAULT( getControllerNh(), "goal_angular_tollerance", goal_angular_tollerance_, 5e-3);
  
  listener_.reset( new tf::TransformListener( getControllerNh(), ros::Duration(1.0), true ) );

  CNR_RETURN_TRUE(*m_logger);
}


bool CartMotionController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);

  inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::doStarting(time);

  T_b_t_start_    = m_Tbt;
  T_b_t_goal_     = m_Tbt;

  as_.reset(
        new actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>(
              getControllerNh(),
              action_ns_,
              boost::bind(&CartMotionController::poseGoalCallback,    this,  _1),
              boost::bind(&CartMotionController::poseCancelCallback,  this,  _1),
              false
            )
        );
  as_->start();

  CNR_RETURN_TRUE(*m_logger);
}



bool CartMotionController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  
  stop_thread_ = true;
  if (gh_)
  {
    gh_->setCanceled();
  }

  if (as_thread_.joinable())
  {
    as_thread_.join();
  }
  gh_.reset();
  
  inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::doStopping(time);

  CNR_RETURN_TRUE(*m_logger);
}


bool CartMotionController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  target_twist_ = Eigen::Vector6d::Zero();

  Eigen::VectorXd from_start;
  rosdyn::getFrameDistance(T_b_t_start_, m_Tbt, from_start );

  Eigen::VectorXd to_go;
  rosdyn::getFrameDistance(T_b_t_goal_, m_Tbt, to_go );

  CNR_DEBUG_THROTTLE(*m_logger, 2, "Distance from start: "  << from_start.block(0,0,3,1).transpose()
                                << " theta: "               << from_start.block(3,0,3,1).norm()
                                << " rot dir: "             << from_start.block(0,0,3,1).normalized().transpose() );
  CNR_DEBUG_THROTTLE(*m_logger, 2, "Distance to     go : "  << to_go     .block(0,0,3,1).transpose()
                                << " theta: "               << to_go     .block(3,0,3,1).norm()
                                << " rot dir: "             << to_go     .block(0,0,3,1).normalized().transpose() );

  double tracking_lin_err = to_go.block(0,0,3,1).norm( );
  double tracking_rot_err = to_go.block(3,0,3,1).norm( );

  double dt = period.toSec() < m_sampling_period ? m_sampling_period : period.toSec();
  goal_reached_ = ( tracking_lin_err < goal_linear_tollerance_ ) && ( tracking_rot_err < goal_angular_tollerance_);

  if( goal_reached_ )
  {
    target_twist_.setZero( );
    geometry_msgs::Twist cmd_twist_msgs;
    tf::twistEigenToMsg(target_twist_, cmd_twist_msgs);
    InverseKinematicsQpPosVelEffController::targetTwistCallback(geometry_msgs::TwistPtr( &cmd_twist_msgs ));
  }
  else
  {
    geometry_msgs::Pose T_b_t_goal;
    tf::poseEigenToMsg(T_b_t_goal_, T_b_t_goal);
    InverseKinematicsQpPosVelEffController::targetPoseCallback(geometry_msgs::PosePtr( &T_b_t_goal ));
  }

  CNR_RETURN_BOOL(*m_logger, InverseKinematicsQpPosVelEffController::doUpdate(time, period));
}

void CartMotionController::poseGoalCallback(actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>::GoalHandle gh)
{
  auto goal= gh.getGoal();

  if( goal->linear_speed < 0 )
  {
    CNR_DEBUG_THROTTLE(*m_logger, 2, "The goal linear speed should be greater or equal to zero while the value "
                                  << goal->linear_speed << " is got");
    CNR_DEBUG_THROTTLE(*m_logger, 2, "The goal linear speed is superimposed equal to zero for safety");
  }
  if( goal->angular_speed < 0 )
  {
    CNR_DEBUG_THROTTLE(*m_logger, 2, "The goal angular speed should be greater or equal to zero while the value "
                                  << goal->angular_speed << " is got");
    CNR_DEBUG_THROTTLE(*m_logger, 2, "The goal angular speed is superimposed equal to zero for safety");
  }

  double goal_linear_speed  = goal->linear_speed  < 0                   ? 0.0
                            : goal->linear_speed  > m_max_cart_lin_acc  ? m_max_cart_lin_acc
                            : goal->linear_speed;
  double goal_angular_speed = goal->angular_speed < 0                   ? 0.0
                            : goal->angular_speed > m_max_cart_ang_vel  ? m_max_cart_ang_vel
                            : goal->angular_speed;

  goal_reached_             = false;
  goal_displacement_        = Eigen::Matrix<double,7,1>( goal->goal.data( ) );
  goal_linear_tollerance_   = goal->linear_tollerance  > 0.0005 ? goal->linear_tollerance  : 0.0005; // to avoid zero
  goal_angular_tollerance_  = goal->angular_tollerance > 0.0050 ? goal->angular_tollerance : 0.0050; // to avoid zero

  if( std::string(goal->goal_link).empty() )
  {
    ROS_FATAL("The 'goal_frame' field is equal to '%s' and it is not correct ....", goal->goal_link.c_str() );
    gh_->setRejected();
    return;
  }
  if( std::string(goal->goal_reference_frame).empty() )
  {
    ROS_FATAL("The 'goal_reference_frame' field is equal to '%s' and it is not correct ....", goal->goal_reference_frame.c_str() );
    gh_->setRejected();
    return;
  }

  goal_linear_speed         = ( goal_linear_speed * m_sampling_period ) > goal_linear_tollerance_
                            ? 0.9 * goal_linear_tollerance / m_sampling_period
                            : goal_linear_speed;
  goal_angular_speed        = ( goal_angular_speed * m_sampling_period ) > goal_angular_tollerance_
                            ? 0.9 * goal_angular_tollerance / m_sampling_period
                            : goal_angular_speed_;
  goal_link_                = boost::iequals( goal->goal_link, "tool" ) ? m_tool_link;
                            : boost::iequals( goal->goal_link, "base" ) ? m_base_link;
                            : goal->goal_link;
  goal_reference_frame_     = boost::iequals( goal->goal_reference_frame, "tool" ) ? m_tool_link
                            : boost::iequals( goal->goal_reference_frame, "base" ) ? m_base_link
                            : goal->goal_reference_frame;
  CNR_DEBUG(*m_logger, " goal_reference_frame_: " << goal_reference_frame_
                    << ", m_tool_link"            << m_tool_link
                    << ", m_base_link"            << m_base_link );

  T_b_t_start_              = m_Tbt;


  Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", "[", "]");
  std::cout.precision(6);
  std::cout.width (7);

  // 	Quaternion (const Scalar &w, const Scalar &x, const Scalar &y, const Scalar &z)
  ROS_DEBUG_STREAM( "goal_displacement: "<< std::fixed << goal_displacement_.transpose().format(CleanFmt) << " in frame {" << goal_reference_frame_ <<"}");
  ROS_DEBUG_STREAM( "T_b_t_start: " << " [ b={" << m_base_link << "}, t={" << m_tool_link << "} ]\n"     << std::fixed << T_b_t_start_.matrix().format(CleanFmt)  );

  // --------------------------------------------------------------------------
  // Whre is the "goal" link at the start event
  Eigen::Affine3d T_b_g_start = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_t_g_start = Eigen::Affine3d::Identity();
  if ( goal_link_ == m_tool_link )
  {
    T_b_g_start = T_b_t_start_;
  }
  else if ( goal_link_ == m_base_link )
  {
    ROS_FATAL("You cannot move the base link of the chain but only a child link ....");
    gh_->setRejected();
  }
  else
  {
    tf::StampedTransform TF_T_b_g;
    ROS_INFO_STREAM("waiting for transform between "<<m_base_link<<" and "<<goal_link_);
    listener_->waitForTransform ( m_base_link, goal_link_, ros::Time(0), ros::Duration ( 10.0 ) );
    listener_->lookupTransform  ( m_base_link, goal_link_, ros::Time(0), TF_T_b_g);
    tf::transformTFToEigen(TF_T_b_g, T_b_g_start);
  }
  T_t_g_start = T_b_t_start_.inverse() * T_b_g_start;
  ROS_DEBUG_STREAM( "T_b_t_start:\n"      << std::fixed << T_b_g_start.matrix().format(CleanFmt) );
  ROS_DEBUG_STREAM( "T_t_g_start:\n"      << std::fixed << T_t_g_start.matrix().format(CleanFmt) );
  // --------------------------------------------------------------------------



  // --------------------------------------------------------------------------
  Eigen::Vector3d     p_gr ( goal_displacement_(0), goal_displacement_(1), goal_displacement_(2) );
  Eigen::Quaterniond  q_gr ( goal_displacement_(6), goal_displacement_(3), goal_displacement_(4), goal_displacement_(5) );
  Eigen::Matrix3d     Q_gr = Eigen::Affine3d( q_gr ).linear();     // rototransaltion to be applied to the goal frame expressed in frame {goal}
  Eigen::Matrix3d     Q_b  = Eigen::Matrix3d::Identity();
  Eigen::Vector3d     DT_b = Eigen::Vector3d::Zero();

  if ( goal_reference_frame_ == m_tool_link )
  {
    Eigen::Affine3d T_t_gr = Eigen::Affine3d::Identity();
    if( is_relative_ )
    {
      Q_b  = T_b_t_start_.linear() * T_t_gr.linear() * Q_gr *T_t_gr.linear().inverse() * T_b_t_start_.linear().inverse();
      DT_b = T_b_t_start_.linear() * T_t_gr.linear() * p_gr;
    }
    else
    {
      assert(0);
    }
  }
  else if ( goal_reference_frame_ == m_base_link )
  {
    ROS_DEBUG_STREAM( "goal_reference_frame_ == m_base_link " );
    if( is_relative_ )
    {
      Q_b  = Q_gr;  // T_b_g_start * Q_g * T_b_g_start.inverse(); // now, everything back to the base frame
      DT_b = p_gr;
    }
    else
    {
      assert(0);
    }
  }
  else
  {
    Eigen::Affine3d       T_b_gr;
    tf::StampedTransform  TF_T_b_gr;
    ROS_INFO_STREAM("waiting for transform between "<<m_base_link<<" and "<<goal_reference_frame_);
    listener_->waitForTransform ( m_base_link, goal_reference_frame_, ros::Time(0), ros::Duration ( 10.0 ) );
    listener_->lookupTransform  ( m_base_link, goal_reference_frame_, ros::Time(0), TF_T_b_gr);
    tf::transformTFToEigen(TF_T_b_gr, T_b_gr);

    Eigen::Affine3d    T_g_gr = T_b_g_start.inverse() * T_b_gr;
    if( is_relative_ )
    {
      Q_b = T_b_gr.linear() * Q_gr * T_b_gr.linear().inverse();
      DT_b = T_b_gr * p_gr;
    }
    else
    {
      assert(0);
    }
  }
  // -------------------------------------


  // -------------------------------------
  T_b_t_goal_.linear() = Q_b * T_b_g_start.linear();
  T_b_t_goal_.translation() += DT_b;
  ROS_DEBUG_STREAM( "T_b_t_start:\n"      << std::fixed << T_b_t_start_.matrix().format(CleanFmt)  );
  ROS_DEBUG_STREAM( "Q_b        :\n"      << std::fixed << Q_b.matrix().format(CleanFmt)  );
  ROS_DEBUG_STREAM( "DT_b       :\n"      << std::fixed << DT_b.transpose().format(CleanFmt)  );
  ROS_DEBUG_STREAM( "T_b_t_goal :\n"      << std::fixed << T_b_t_goal_.matrix().format(CleanFmt)  );
  //====================
  // Now we can accept the execution of the callback
  std::shared_ptr<actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>::GoalHandle> current_gh;
  current_gh.reset(new actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>::GoalHandle(gh));
  gh_ = current_gh;

  ROS_DEBUG("Starting managing new goal");
  gh_->setAccepted();
  stop_thread_ = true;
  if (as_thread_.joinable())
  {
    as_thread_.join();
  }
  //====================

  stop_thread_  = false;
  as_thread_ = std::thread(&CartMotionController::poseThread,this);
  
}

void CartMotionController::poseCancelCallback(actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>::GoalHandle gh)
{
  CNR_TRACE_START(*m_logger, "Stopping active goal");
  if (gh_)
  {
    stop_thread_ = true;
    gh_->setCanceled();
    if (as_thread_.joinable())
    {
      as_thread_.join();
    }
    gh_.reset();
  }
  else
  {
    CNR_RETURN_NOTOK(*m_logger, void(), "no goal to cancel");
  }
  CNR_RETURN_OK(*m_logger, void());
}

void CartMotionController::poseThread()
{
  CNR_TRACE_START(*m_logger, "START ACTION GOAL LOOPING");
  ros::WallRate lp(100);

  while (getControllerNh().ok())
  { 
    if (stop_thread_)
    {
      CNR_WARN(*m_logger, "The robot's not reached the goal, but a stop has been triggered.");
      break;
    }
    
    lp.sleep();
    if (!gh_)
    {
      ROS_ERROR("Goal handle is not initialized");
      break;
    }
    
    cartesian_motion_msgs::cartMotionResult result;
    if ( goal_reached_ )
    {
      CNR_INFO(*m_logger, "The robot's reached the goal"  );
      result.error_code=0;
      result.error_string="finished";
      gh_->setSucceeded(result);
      break;
    }
  }
  gh_.reset();
  CNR_RETURN_OK(*m_logger, void());
}


}
}

