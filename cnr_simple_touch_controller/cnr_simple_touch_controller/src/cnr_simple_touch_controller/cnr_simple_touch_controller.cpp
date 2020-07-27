#include <locale>
#include <cnr_simple_touch_controller/cnr_simple_touch_controller.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <boost/algorithm/string.hpp>


/**
 * @brief PLUGINLIB_EXPORT_CLASS
 */
PLUGINLIB_EXPORT_CLASS(itia::control::SimpleTouchController, controller_interface::ControllerBase);


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


namespace itia
{
namespace control
{


bool SimpleTouchController::init(hardware_interface::ForceTorqueInterface *hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_hw            = hw;
  m_root_nh       = root_nh;
  m_controller_nh = controller_nh;
  m_controller_nh.setCallbackQueue(&m_controller_nh_callback_queue);

  bool debug = false;
  GET_AND_DEFAULT( m_controller_nh, "debug", debug, false );
  if (debug)
  {
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  //** PARAMS UNDER CONTROLLER PARAM NAMESPACE*********************************************
  std::string output_twist_name;

  GET_AND_RETURN ( m_controller_nh, "sensor_name"   , m_ft_resource_name );
  GET_AND_RETURN ( m_controller_nh, "base_link"     , m_base_frame    );
  GET_AND_RETURN ( m_controller_nh, "tool_link"     , m_tool_frame    );
  GET_AND_RETURN ( m_controller_nh, "sensor_frame"  , m_sensor_frame  );
  GET_AND_DEFAULT( m_controller_nh, "output_twist_ns" , output_twist_name, m_controller_nh.getNamespace()+"/target_cart_twist" );

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



  m_target_twist_pub = m_controller_nh.advertise<geometry_msgs::TwistStamped>(output_twist_name,1000);

  m_listener.reset( new tf::TransformListener( m_controller_nh ) );
  return true;
}


void SimpleTouchController::starting(const ros::Time& time)
{
  ROS_INFO("[ %s ] Starting controller",  m_controller_nh.getNamespace().c_str());

  m_touched = false;

  Eigen::Vector3d force ( m_ft_h.getForce( ) );
  Eigen::Vector3d torque( m_ft_h.getForce( ) );

  m_target_twist.setZero( );


  m_controller_nh_callback_queue.callAvailable();

  m_as.reset(new actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>(m_controller_nh, "simple_touch",
                                                                      boost::bind(&SimpleTouchController::actionGoalCallback,    this,  _1), 
                                                                      boost::bind(&SimpleTouchController::actionCancelCallback,  this,  _1), 
                                                                      false));
  m_as->start();

  ROS_INFO("[ %s ] Controller Started",  m_controller_nh.getNamespace().c_str());

}


void SimpleTouchController::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s ] Stopping Controller",  m_controller_nh.getNamespace().c_str());
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

    m_target_twist_pub.publish( tw );

  }

  m_controller_nh_callback_queue.callAvailable();
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
  
  ROS_INFO("[ %s ] Controller Succesfully Stopped",  m_controller_nh.getNamespace().c_str());
  return;
}



void SimpleTouchController::update(const ros::Time& time, const ros::Duration& period)
{
  
  m_wrench_s.block(0,0,3,1) = Eigen::Vector3d( m_ft_h.getForce( ) );
  m_wrench_s.block(3,0,3,1) = Eigen::Vector3d( m_ft_h.getTorque( ) );


  // ROS_DEBUG_STREAM_THROTTLE(2, "[ " << m_controller_nh.getNamespace() <<  "] Measured Wrench {" << m_sensor_frame<<"}: " << m_wrench_s.transpose() );
  m_controller_nh_callback_queue.callAvailable();

  if(m_touched)
  {
    ROS_DEBUG_STREAM_THROTTLE(2, "[ " << m_controller_nh.getNamespace() <<  "] Touched! Set to zero the output twist" );
    m_target_twist.setZero();
  }
  
  ROS_DEBUG_STREAM_THROTTLE(2, "[ " << m_controller_nh.getNamespace() <<  "] Output twist: " << m_target_twist.transpose() );

  geometry_msgs::TwistStamped tw;

  tw.twist.linear .x = m_target_twist(0,0);
  tw.twist.linear .y = m_target_twist(1,0);
  tw.twist.linear .z = m_target_twist(2,0);
  tw.twist.angular.x = m_target_twist(3,0);
  tw.twist.angular.y = m_target_twist(4,0);
  tw.twist.angular.z = m_target_twist(5,0);
  tw.header.frame_id = m_goal_twist_frame;
  tw.header.stamp    = ros::Time::now();

  m_target_twist_pub.publish(tw);


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

    ROS_INFO("[ %s ] New Goal Received, action start!",  m_controller_nh.getNamespace().c_str());
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
      ROS_DEBUG_STREAM( "[ " << m_controller_nh.getNamespace() <<  "] Goal goal wrench        {" << m_goal_wrench_frame <<"} " << m_goal_wrench_g       .transpose() );
      ROS_DEBUG_STREAM( "[ " << m_controller_nh.getNamespace() <<  "] Goal goal wrench toll   {" << m_goal_wrench_frame <<"} " << m_goal_wrench_toll    .transpose() );
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

    ROS_DEBUG_STREAM( "[ " << m_controller_nh.getNamespace() <<  "] Goal goal wrench deadband {" << m_goal_wrench_frame <<"} " << m_goal_wrench_deadband.transpose() );
    ROS_DEBUG_STREAM( "[ " << m_controller_nh.getNamespace() <<  "] Goal goal wrench frame    {" << m_goal_wrench_frame <<"} " << m_goal_wrench_frame                );
    ROS_DEBUG_STREAM( "[ " << m_controller_nh.getNamespace() <<  "] Goal goal twist           {" << m_goal_twist_frame  <<"} " << m_goal_twist          .transpose() );
    ROS_DEBUG_STREAM( "[ " << m_controller_nh.getNamespace() <<  "] Goal goal twist frame     {" << m_goal_wrench_frame <<"} " << m_goal_twist_frame                 );

    m_touched = false;
    m_stop_thread  = false;
    m_as_thread = std::thread(&SimpleTouchController::actionThreadTunction,this);
  }
  catch( std::exception& e )
  {
    ROS_ERROR_STREAM("Exception at line: " << l << " what: " << e.what() );
    assert(0);
  }
  catch( ... )
  {
    ROS_ERROR_STREAM("Exception at line: " << l);
    assert(0);
  }
  
}

void SimpleTouchController::actionCancelCallback(actionlib::ActionServer< simple_touch_controller_msgs::simpleTouchAction >::GoalHandle gh)
{
  ROS_DEBUG("[ %s ] Triggered the Cancel of the Action",  m_controller_nh.getNamespace().c_str());
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
    ROS_WARN("[ %s ] Triggered the Cancel of the Action but none Goal is active.",  m_controller_nh.getNamespace().c_str());
  }
  ROS_DEBUG("[ %s ] Action Succesfully Cancelled",  m_controller_nh.getNamespace().c_str());
}

void SimpleTouchController::actionThreadTunction()
{
  ROS_DEBUG("[ %s ] START ACTION GOAL LOOP",  m_controller_nh.getNamespace().c_str());
  ros::WallRate lp(100);

  while (m_controller_nh.ok())
  {
    lp.sleep();
    if (!m_gh)
    {
      ROS_ERROR("[ %s ] Goal handle is not initialized",  m_controller_nh.getNamespace().c_str());
      break;
    }

    if( m_stop_thread )
    {
      ROS_ERROR("[ %s ] Triggered an external stop. Break the action loop.",  m_controller_nh.getNamespace().c_str());
      break;
    }

    Eigen::Affine3d T_gs;

    tf::StampedTransform TF_T_bs, TF_T_bt;
    ROS_DEBUG_STREAM_THROTTLE( 2, "[ " << m_controller_nh.getNamespace() <<  "] Listening for TF transormation between '" << m_base_frame << "' and '" << m_sensor_frame  <<"'" );
    m_listener->waitForTransform ( m_base_frame, m_sensor_frame       , ros::Time(0), ros::Duration ( 10.0 ) );
    m_listener->lookupTransform  ( m_base_frame, m_sensor_frame       , ros::Time(0), TF_T_bs);

    ROS_DEBUG_STREAM_THROTTLE( 2, "[ " << m_controller_nh.getNamespace() <<  "] Listening for TF transormation between '" << m_base_frame << "' and '" << m_tool_frame  <<"'" );
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
      ROS_DEBUG_STREAM_THROTTLE(2, "[ " << m_controller_nh.getNamespace() <<  "] Goal     Force {" << m_goal_wrench_frame  <<"}: " << goal_force_g.transpose() <<", norm:       " << goal_force_g.norm() );
      ROS_DEBUG_STREAM_THROTTLE(2, "[ " << m_controller_nh.getNamespace() <<  "] Measured Force {" << m_goal_wrench_frame  <<"}: " << force_g     .transpose() << " projection: " << force_g.dot( goal_force_g.normalized()) );

      Eigen::Vector3d tracking_force_error  = (goal_force_g.norm() - force_g.dot( goal_force_g.normalized()) )
                                            *  goal_force_g.normalized();

      ROS_DEBUG_STREAM_THROTTLE( 2, "[ " << m_controller_nh.getNamespace() <<  "] Tracking force error {"<< m_goal_wrench_frame <<" }: "<<tracking_force_error.transpose());
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
      ROS_INFO_STREAM("Measured wrench in {s} while stopping : " << m_wrench_s.transpose());
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
