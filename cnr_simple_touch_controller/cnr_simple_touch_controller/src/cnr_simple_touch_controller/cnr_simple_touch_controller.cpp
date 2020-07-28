#include <locale>
#include <controller_interface/controller.h>
#include <cnr_simple_touch_controller/cnr_simple_touch_controller.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <boost/algorithm/string.hpp>


/**
 * @brief PLUGINLIB_EXPORT_CLASS
 */
PLUGINLIB_EXPORT_CLASS(cnr::control::SimpleTouchController, controller_interface::ControllerBase);


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
    ROS_ERROR("[ %s ] The param '%s/%s' is not defined", nh.getNamespace().c_str(), nh.getNamespace().c_str(), std::string( param ).c_str() );\
    return false;\
  }



#define GET_AND_DEFAULT( nh, param, value, def )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_WARN("[ %s ] The param '%s/%s' is not defined", nh.getNamespace().c_str(), nh.getNamespace().c_str(), std::string( param ).c_str() );\
    ROS_WARN("[ %s ] Default value '%s' superimposed. ", nh.getNamespace().c_str(), std::to_string( def ).c_str() );\
    value=def;\
  }


namespace cnr
{
namespace control
{


bool SimpleTouchController::doInit( )
{
  //** PARAMS UNDER CONTROLLER PARAM NAMESPACE*********************************************
  std::string output_twist_name;

  GET_AND_RETURN ( getControllerNh(), "sensor_name"   , m_ft_resource_name );
  GET_AND_RETURN ( getControllerNh(), "base_link"     , m_base_frame    );
  GET_AND_RETURN ( getControllerNh(), "tool_link"     , m_tool_frame    );
  GET_AND_RETURN ( getControllerNh(), "sensor_frame"  , m_sensor_frame  );
  GET_AND_DEFAULT( getControllerNh(), "output_twist_ns" , output_twist_name, getControllerNamespace()+"/target_cart_twist" );

  m_goal_wrench_frame = m_sensor_frame;
  m_goal_twist_frame  = m_tool_frame;

  m_goal_wrench_norm = -1;
  m_goal_wrench_norm_toll = -1;
  m_wrench_s             .setZero( );
  m_target_twist         .setZero( );
  m_goal_wrench_g        .setZero( );
  m_goal_wrench_toll     .setZero( );
  m_goal_wrench_deadband .setZero( );
  m_goal_twist           .setZero( );

  m_ft_h = m_hw->getHandle( m_ft_resource_name );



  add_publisher<geometry_msgs::TwistStamped>("target_twist", output_twist_name,1000);

  m_listener.reset( new tf::TransformListener( getControllerNh() ) );
  return true;
}


bool SimpleTouchController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger, "Starting controller");

  m_touched = false;

  Eigen::Vector3d force ( m_ft_h.getForce( ) );
  Eigen::Vector3d torque( m_ft_h.getForce( ) );

  m_target_twist.setZero( );

  m_as.reset(new actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>(getControllerNh(), "simple_touch",
                                                                      boost::bind(&SimpleTouchController::actionGoalCallback,    this,  _1), 
                                                                      boost::bind(&SimpleTouchController::actionCancelCallback,  this,  _1), 
                                                                      false));
  m_as->start();

  CNR_RETURN_TRUE(*m_logger, "Controller Started");
}


bool SimpleTouchController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger, "Stopping Controller");
  m_target_twist.setZero();

  {
    geometry_msgs::TwistStamped tw;

    tw.twist.linear .x = m_target_twist(0,0);
    tw.twist.linear .y = m_target_twist(1,0);
    tw.twist.linear .z = m_target_twist(2,0);
    tw.twist.angular.x = m_target_twist(3,0);
    tw.twist.angular.y = m_target_twist(4,0);
    tw.twist.angular.z = m_target_twist(5,0);

    tw.header.frame_id = m_sensor_frame;
    tw.header.stamp    = ros::Time::now();

    publish( "target_twist", tw );

  }
  if (m_gh)
  {
    m_gh->setCanceled();
  }
  m_stop_thread = true;
  if (m_as_thread.joinable())
  {
    m_as_thread.join();
  }
  m_gh.reset();
  
  CNR_RETURN_TRUE(*m_logger, "Controller Succesfully Stopped");
}



bool SimpleTouchController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE(*m_logger,5.0);
  m_wrench_s.block(0,0,3,1) = Eigen::Vector3d( m_ft_h.getForce( ) );
  m_wrench_s.block(3,0,3,1) = Eigen::Vector3d( m_ft_h.getTorque( ) );

  if(m_touched)
  {
    CNR_DEBUG_THROTTLE(*m_logger, 2.0, "[ " << getControllerNamespace() <<  "] Touched! Set to zero the output twist" );
    m_target_twist.setZero();
  }
  
  CNR_DEBUG_THROTTLE(*m_logger, 2.0, "[ " << getControllerNamespace() <<  "] Output twist: " << m_target_twist.transpose() );

  geometry_msgs::TwistStamped tw;

  tw.twist.linear .x = m_target_twist(0,0);
  tw.twist.linear .y = m_target_twist(1,0);
  tw.twist.linear .z = m_target_twist(2,0);
  tw.twist.angular.x = m_target_twist(3,0);
  tw.twist.angular.y = m_target_twist(4,0);
  tw.twist.angular.z = m_target_twist(5,0);
  tw.header.frame_id = m_goal_twist_frame;
  tw.header.stamp    = ros::Time::now();

  publish("target_twist", tw);

  CNR_RETURN_TRUE_THROTTLE(*m_logger,5.0);

}

void SimpleTouchController::actionGoalCallback(actionlib::ActionServer< simple_touch_controller_msgs::simpleTouchAction>::GoalHandle gh)
{
  size_t l = __LINE__;
  try
  {
    auto goal = gh.getGoal();

    std::shared_ptr<actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>::GoalHandle> current_gh;

    current_gh.reset(new actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>::GoalHandle(gh));
    m_gh = current_gh;

    CNR_INFO(*m_logger,"New Goal Received, action start!");
    m_gh->setAccepted();


    if (m_as_thread.joinable())
    {
      m_as_thread.join();
    }


    m_goal_wrench_frame     = goal->target_wrench_frame;
    m_goal_wrench_deadband  = Eigen::Vector6d( goal->wrench_deadband.data() );
    if( goal->target_wrench.size() == 1 )
    {
      m_goal_wrench_norm      = std::fabs( goal->target_wrench.front()  );
      m_goal_wrench_norm_toll = std::fabs( goal->wrench_toll.front()    );
    }
    else
    {
      m_goal_wrench_norm      = -1;
      m_goal_wrench_g         = Eigen::Vector6d( goal->target_wrench.data() );
      m_goal_wrench_toll      = Eigen::Vector6d( goal->wrench_toll.data() ).cwiseAbs( );

      m_goal_wrench_frame     = goal->target_wrench_frame;
      CNR_DEBUG(*m_logger,"[ " << getControllerNamespace() <<  "] Goal goal wrench        {" << m_goal_wrench_frame <<"} " << m_goal_wrench_g       .transpose() );
      CNR_DEBUG(*m_logger,"[ " << getControllerNamespace() <<  "] Goal goal wrench toll   {" << m_goal_wrench_frame <<"} " << m_goal_wrench_toll    .transpose() );
    }


    std::string frame_id = boost::to_lower_copy( goal->goal_twist_frame);
    if( frame_id == "tool")
    {
      m_goal_twist_frame      = m_tool_frame;
    }
    else if( frame_id == "base" )
    {
      m_goal_twist_frame      = m_base_frame;
    }
    else
    {
      m_goal_twist_frame  =  goal->goal_twist_frame;
    }
    m_goal_twist            = Eigen::Vector6d( goal->goal_twist.data() );

    CNR_DEBUG(*m_logger,"[ " << getControllerNamespace() <<  "] Goal goal wrench deadband {" << m_goal_wrench_frame <<"} " << m_goal_wrench_deadband.transpose() );
    CNR_DEBUG(*m_logger,"[ " << getControllerNamespace() <<  "] Goal goal wrench frame    {" << m_goal_wrench_frame <<"} " << m_goal_wrench_frame                );
    CNR_DEBUG(*m_logger,"[ " << getControllerNamespace() <<  "] Goal goal twist           {" << m_goal_twist_frame  <<"} " << m_goal_twist          .transpose() );
    CNR_DEBUG(*m_logger,"[ " << getControllerNamespace() <<  "] Goal goal twist frame     {" << m_goal_wrench_frame <<"} " << m_goal_twist_frame                 );

    m_touched = false;
    m_stop_thread  = false;
    m_as_thread = std::thread(&SimpleTouchController::actionThreadTunction,this);
  }
  catch( std::exception& e )
  {
    CNR_ERROR(*m_logger, "Exception at line: " << l << " what: " << e.what() );
    assert(0);
  }
  catch( ... )
  {
    CNR_ERROR(*m_logger, "Exception at line: " << l);
    assert(0);
  }
  
}

void SimpleTouchController::actionCancelCallback(actionlib::ActionServer< simple_touch_controller_msgs::simpleTouchAction >::GoalHandle gh)
{
  CNR_DEBUG(*m_logger, "Triggered the Cancel of the Action");
  if (m_gh)
  {
    m_gh->setCanceled();
    m_stop_thread = true;
    if (m_as_thread.joinable())
    {
      m_as_thread.join();
    }
    m_gh.reset();
    m_target_twist.setZero( );
  }
  else
  {
    ROS_WARN("[ %s ] Triggered the Cancel of the Action but none Goal is active.",  getControllerNamespace().c_str());
  }
  CNR_DEBUG(*m_logger, "Action Succesfully Cancelled");
}

void SimpleTouchController::actionThreadTunction()
{
  CNR_DEBUG(*m_logger, "START ACTION GOAL LOOP");
  ros::WallRate lp(100);

  while (getControllerNh().ok())
  {
    lp.sleep();
    if (!m_gh)
    {
      CNR_ERROR(*m_logger, "Goal handle is not initialized");
      break;
    }

    if( m_stop_thread )
    {
      CNR_ERROR(*m_logger, "Triggered an external stop. Break the action loop.");
      break;
    }

    Eigen::Affine3d T_gs;

    tf::StampedTransform TF_T_bs, TF_T_bt;
    CNR_DEBUG_THROTTLE( *m_logger, 2, "[ " << getControllerNamespace() <<  "] Listening for TF transormation between '" << m_base_frame << "' and '" << m_sensor_frame  <<"'" );
    m_listener->waitForTransform ( m_base_frame, m_sensor_frame       , ros::Time(0), ros::Duration ( 10.0 ) );
    m_listener->lookupTransform  ( m_base_frame, m_sensor_frame       , ros::Time(0), TF_T_bs);

    CNR_DEBUG_THROTTLE( *m_logger, 2, "[ " << getControllerNamespace() <<  "] Listening for TF transormation between '" << m_base_frame << "' and '" << m_tool_frame  <<"'" );
    m_listener->waitForTransform ( m_base_frame, m_tool_frame         , ros::Time(0), ros::Duration ( 10.0 ) );
    m_listener->lookupTransform  ( m_base_frame, m_tool_frame         , ros::Time(0), TF_T_bt);

    Eigen::Affine3d T_bs; tf::transformTFToEigen(TF_T_bs, T_bs);
    Eigen::Affine3d T_bt; tf::transformTFToEigen(TF_T_bt, T_bt);
    if( m_goal_wrench_frame  == "tool"  || m_goal_wrench_frame  == "endeffector" )
    {
      Eigen::Affine3d T_bg = T_bt;
      T_gs =T_bg.inverse() * T_bs;
    }
    else if(m_goal_wrench_frame  == "base" )
    {
      T_gs = T_bs;
    }
    else
    {
      tf::StampedTransform TF_T_bg;
      m_listener->waitForTransform ( m_base_frame, m_goal_wrench_frame, ros::Time(0), ros::Duration ( 10.0 ) );
      m_listener->lookupTransform  ( m_base_frame, m_goal_wrench_frame, ros::Time(0), TF_T_bg);

      Eigen::Affine3d T_bg; tf::transformTFToEigen(TF_T_bg, T_bg);
      T_gs = T_bg.inverse() * T_bs;
    }

    Eigen::Vector3d force_g = T_gs.inverse() * Eigen::Vector3d( m_wrench_s.block(0,0,3,1) );
    for( size_t i=0; i<3; i++)
    {
      force_g(i) = force_g(i) >  m_goal_wrench_deadband(i) ? force_g(i) - m_goal_wrench_deadband(i)
                 : force_g(i) < -m_goal_wrench_deadband(i) ? force_g(i) + m_goal_wrench_deadband(i)
                 : 0.0;
    }

    bool target_achieved = false;
    if( m_goal_wrench_norm < 0)
    {
      Eigen::Vector3d goal_force_g  = m_goal_wrench_g.block(0,0,3,1);
      CNR_DEBUG_THROTTLE(*m_logger, 2.0, "[ " << getControllerNamespace() <<  "] Goal     Force {" << m_goal_wrench_frame  <<"}: " << goal_force_g.transpose() <<", norm:       " << goal_force_g.norm() );
      CNR_DEBUG_THROTTLE(*m_logger, 2.0, "[ " << getControllerNamespace() <<  "] Measured Force {" << m_goal_wrench_frame  <<"}: " << force_g     .transpose() << " projection: " << force_g.dot( goal_force_g.normalized()) );

      Eigen::Vector3d tracking_force_error  = (goal_force_g.norm() - force_g.dot( goal_force_g.normalized()) )
                                            *  goal_force_g.normalized();

      CNR_DEBUG_THROTTLE( *m_logger, 2, "[ " << getControllerNamespace() <<  "] Tracking force error {"<< m_goal_wrench_frame <<" }: "<<tracking_force_error.transpose());
      target_achieved = force_g.dot( goal_force_g.normalized() ) > goal_force_g.norm();
    }
    else
    {
      target_achieved = ( force_g.norm()  >m_goal_wrench_norm );
    }

    ///----------------------
    simple_touch_controller_msgs::simpleTouchResult result;
    if ( target_achieved )
    {
      m_touched = true;
      CNR_INFO(*m_logger,"Measured wrench in {s} while stopping : " << m_wrench_s.transpose());
      result.error_code   = 0;
      result.error_string = "finished";
      m_gh->setSucceeded(result);
      m_target_twist.setZero();
      break;
    }
    else
    {
      m_target_twist = m_goal_twist;
    }
    ///----------------------

  }
  m_gh.reset();
}


}
}
