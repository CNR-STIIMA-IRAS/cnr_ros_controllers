#include <boost/algorithm/string.hpp>

#include <geometry_msgs/Pose.h>
#include <state_space_filters/filtered_values.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_cartesian_teleop_controller/cnr_cartesian_teleop_controller.h>

#include <pluginlib/class_list_macros.h>

namespace eu = eigen_utils;
namespace ect = eigen_control_toolbox;

PLUGINLIB_EXPORT_CLASS(cnr::control::CartesianTeleopController  , controller_interface::ControllerBase)

namespace cnr
{
namespace control
{


/**
 * @brief CartesianTeleopController::CartesianTeleopController
 */
inline CartesianTeleopController::CartesianTeleopController()
{
}

/**
 * @brief CartesianTeleopController::doInit
 * @return
 */
inline bool CartesianTeleopController::doInit()
{
  //INIT PUB/SUB
  std::string setpoint_topic_name;
  setpoint_topic_name = this->getControllerNamespace() + "/target_joint_teleop";

  this->template add_subscriber<sensor_msgs::JointState>(
        setpoint_topic_name,5,boost::bind(&CartesianTeleopController::callback,this,_1), false);

  this->setPriority(this->QD_PRIORITY);

  ect::FilteredVectorXd::Value dead_band;
  ect::FilteredVectorXd::Value saturation;
  ect::FilteredVectorXd::Value init_value;

  dead_band = 0.0 * m_chain.getDQMax();
  saturation = m_chain.getDQMax();
  init_value = dead_band;
  if(!m_vel_fitler_sp.activateFilter ( dead_band, saturation, (10.0 / 2.0 / M_PI), this->m_sampling_period, init_value ))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  m_vel_sp = m_vel_fitler_sp.getUpdatedValue();
  m_pos_sp = this->getPosition();

  m_has_pos_sp = false;

  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianTeleopController::doStarting
 * @param time
 */
inline bool CartesianTeleopController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  m_pos_sp = this->getPosition();
  m_vel_sp = 0 * this->getVelocity();
  m_dist_to_pos_sp =  0 * this->getVelocity();
  m_vel_sp_last = m_vel_sp;
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianTeleopController::stopping
 * @param time
 */
inline bool CartesianTeleopController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianTeleopController::doUpdate
 * @param time
 * @param period
 * @return
 */
inline bool CartesianTeleopController::doUpdate(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;

  std::lock_guard<std::mutex> lock(m_mtx);
  rosdyn::VectorXd vel_sp = m_vel_sp;
  rosdyn::VectorXd pos_sp = m_pos_sp;
  if(m_has_pos_sp)
  {
    auto dist_to_sp_perc = eu::norm(m_pos_sp - this->getPosition()) / eu::norm(m_dist_to_pos_sp);
    auto dir_to_sp      = eu::normalized(m_pos_sp - this->getPosition());
    vel_sp = eu::norm(vel_sp) * dist_to_sp_perc * dir_to_sp;
  }
  else
  {
    m_vel_fitler_sp.update(vel_sp);
    m_pos_sp = m_pos_sp + m_vel_fitler_sp.getUpdatedValue()* period.toSec();
    pos_sp   = m_pos_sp;
    if(rosdyn::saturatePosition(this->m_chain,pos_sp, &report))
    {
      CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
    }
    m_pos_sp  = pos_sp;
  }
  this->setCommandPosition( pos_sp );
  this->setCommandVelocity( vel_sp );

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

/**
 * @brief CartesianTeleopController::callback
 * @param msg
 */
inline void CartesianTeleopController::callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  try
  {
    CNR_DEBUG_THROTTLE(this->logger(), 2, "[ " << this->getControllerNamespace() << " ] >>>>>>>>>> TWIST TARGET TARGET RECEIVED!");

    Eigen::Vector6d twist = Eigen::Vector6d::Zero( );
    twist (0) = msg->twist.linear.x;
    twist (1) = msg->twist.linear.y;
    twist (2) = msg->twist.linear.z;
    twist (3) = msg->twist.angular.x;
    twist (4) = msg->twist.angular.y;
    twist (5) = msg->twist.angular.z;

    if(std::isnan(twist.norm()))
    {
      CNR_WARN_THROTTLE( this->logger(), 2, "[ " << this->getControllerNamespace() <<" ] SAFETY CHECK - Received a Twist with nan values... superimposed to zero!" );
      twist = Eigen::Vector6d::Zero();
    }

    CNR_WARN_THROTTLE( this->logger(), 2, "[ " << this->getControllerNamespace() <<" ] Reference Twist {" << msg->header.frame_id << "}     : " << twist.transpose() );

    std::string frame_id = boost::to_lower_copy( msg->header.frame_id);
    if ( frame_id == "tool" )
    {
      m_twist_in_b = rosdyn::spatialRotation( twist, this->chainCommand().toolPose().rotation());
    }
    else if ( frame_id == "base" )
    {
      m_twist_in_b = twist;
    }
    else
    {
      tf::StampedTransform TF_T_bf;
      CNR_DEBUG_THROTTLE(this->logger(), 2, "[ " << this->getControllerNamespace() << " ] listening to transform between "
        <<this->chain().getLinksName().front()<<" and "<<msg->header.frame_id);
      m_listener->waitForTransform ( this->chain().getLinksName().front(), msg->header.frame_id, ros::Time(0), ros::Duration ( 10.0 ) );
      m_listener->lookupTransform  ( this->chain().getLinksName().front(), msg->header.frame_id, ros::Time(0), TF_T_bf);
      Eigen::Affine3d T_bf;
      tf::transformTFToEigen(TF_T_bf, T_bf);

      m_twist_in_b = rosdyn::spatialRotation( twist, T_bf.rotation());
    }
    CNR_DEBUG_THROTTLE( this->logger(), 2, "[ " << m_controller_nh.getNamespace() << " ] Reference Twist {base}     : " << m_twist_in_b.transpose() );
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN_STREAM("[ " << m_controller_nh.getNamespace() << " ] Listening to transform between "<<this->chain().getLinksName().front()<<" and "<<msg->header.frame_id <<" failed" );
    m_twist_in_b = Eigen::Vector6d::Zero();
  }
  catch(std::exception& e)
  {
    ROS_WARN("[ %s ] something wrong in Getting the data from tf at line %zu",  m_controller_nh.getNamespace().c_str(), l);
    ROS_WARN_STREAM("[ " << m_controller_nh.getNamespace() << " ]Exception "<< e.what() );
    m_twist_in_b = Eigen::Vector6d::Zero();
  }
  catch(...)
  {
    ROS_WARN("[ %s ] something wrong in Target Callback at line %zu",  m_controller_nh.getNamespace().c_str(), l);
    m_twist_in_b = Eigen::Vector6d::Zero();
  }
  ROS_DEBUG_STREAM_THROTTLE(2, "[ " <<  m_controller_nh.getNamespace() << " ] <<<<<<<<< TWIST TARGET TARGET RECEIVED!"  );

  return;
}





void CartTargetKinematicFilter::update  ( const ros::Time& time           , const ros::Duration& period
                                        , const Eigen::Affine3d& target_p , const Eigen::Vector6d& target_v
                                        , const Eigen::VectorXd& q        , const Eigen::VectorXd& qd
                                        , Eigen::VectorXd& cmd_q          , Eigen::VectorXd& cmd_qd)
{
  m_last_target_p = m_target_p;
  m_last_target_v = m_target_v;
  double dt = period.toSec() >= m_dt ? period.toSec() : m_dt;


  try
  {
    Eigen::Affine3d actual_p = m_chain->getTransformation( q );
    // ==============================
    ROS_DEBUG_ONCE( "[ CartTargetKinematicFilter ] Set Target according to Priority");
    Eigen::VectorXd distance;
    Eigen::Vector6d nominal_v; nominal_v.setZero();
    if( m_priority == JointTargetKinematicFilter::Q_PRIORITY )
    {
      ROS_DEBUG_ONCE( "[ CartTargetKinematicFilter ] Position Priority");
      rosdyn::getFrameDistance(m_target_p,m_last_target_p, distance);

      nominal_v = distance / dt;
    }
    else if( m_priority == JointTargetKinematicFilter::QD_PRIORITY )
    {
      ROS_DEBUG_ONCE( "[ CartTargetKinematicFilter ] Velocity Priority ");
      nominal_v = target_v;
      if(std::isnan(nominal_v.norm()))
      {
        ROS_WARN_THROTTLE( 2, "[ CartTargetKinematicFilter ] SAFETY CHECK - Received a velocity with nan values... superimposed to zero!");
        nominal_v.setZero();
      }
    }
    //ROS_DEBUG_STREAM_THROTTLE(2,  "[ CartTargetKinematicFilter ] Nominal Velocity: " << nominal_v.transpose() );
    // ==============================


    // ==============================
    ROS_DEBUG_ONCE( "Saturate the velocity");
    Eigen::Vector6d twist = Eigen::Vector6d::Zero();
    if( nominal_v.norm() > 0 )
    {
      twist = nominal_v.block(0,0,3,1).norm() > m_max_cart_lin_vel ? nominal_v * m_max_cart_lin_vel / ( nominal_v.block(0,0,3,1).norm()   ) : nominal_v;
      twist = twist.block(3,0,3,1).norm()     > m_max_cart_ang_vel ? twist     * m_max_cart_ang_vel / ( twist.block(3,0,3,1).norm() ) : twist;
    }
    //ROS_DEBUG_STREAM_THROTTLE(2,  "[ CartTargetKinematicFilter ] Saturated Velocity (Max Speed Saturation): " << twist.transpose() );

    Eigen::Vector6d dtwist = (twist - m_last_target_v)/dt;
    if(std::isnan(dtwist.norm()))
    {
      ROS_WARN_THROTTLE( 2, "[ CartTargetKinematicFilter ] SAFETY CHECK - Received a Twist with nan values... superimposed to zero!");
      dtwist = Eigen::Vector6d::Zero();
    }

    if( dtwist.norm() > 0 )
    {
      dtwist *= dtwist.block(0,0,3,1).norm() > m_max_cart_lin_acc ? m_max_cart_lin_acc/dtwist.block(0,0,3,1).norm() : 1.0;
      dtwist *= dtwist.block(3,0,3,1).norm() > m_max_cart_ang_acc ? m_max_cart_ang_acc/dtwist.block(3,0,3,1).norm() : 1.0;
    }

    twist = m_last_target_v + dtwist * dt;
    //ROS_DEBUG_STREAM_THROTTLE(2,  "[ CartTargetKinematicFilter ] Saturated Velocity (Max Acc Saturation): " << twist.transpose() );

    Eigen::VectorXd  target_q = cmd_q;
    Eigen::VectorXd  target_qd = Eigen::VectorXd( m_nAx ).setZero();
    Eigen::Matrix6Xd J = m_chain->getJacobian( cmd_q );

    Eigen::FullPivLU<Eigen::MatrixXd> pinv_J( J );
    pinv_J.setThreshold(1e-2);
    target_qd = pinv_J.solve(twist);
    if (pinv_J.rank()< std::min( 6, int(q.rows()) ) )
    {
      Eigen::MatrixXd null=pinv_J.kernel();
      ROS_WARN_THROTTLE(2,"[ CartTargetKinematicFilter ] Singolarity point! Rank equal to %zu, while the expected dimension is %d", pinv_J.rank(), std::min( int(q.rows()), 6 ) );

      for (int iC=0;iC<null.cols();iC++)
      {
        Eigen::VectorXd null_versor=null.col(iC);
        null_versor.normalize();
        target_qd = target_qd - (target_qd.dot(null_versor))*null_versor;
      }
    }

    if(std::isnan(target_qd.norm()))
    {
      ROS_WARN_THROTTLE( 2, "[ CartTargetKinematicFilter ] SAFETY CHECK - Target Velocity with nan values... superimposed to zero!");
      target_qd = Eigen::Vector6d::Zero();
    }
    target_q = target_qd + cmd_q;

    //ROS_DEBUG_STREAM_THROTTLE(2,  "[ CartTargetKinematicFilter ] Target Joint Pos (before joint saturation)"  << target_q .transpose() );
    //ROS_DEBUG_STREAM_THROTTLE(2,  "[ CartTargetKinematicFilter ] Target Joint Vel (before joint saturation)"  << target_qd.transpose() );

    m_filter.update(time, period, target_q, target_qd, q, qd, cmd_q, cmd_qd);

    //ROS_DEBUG_STREAM_THROTTLE(2,  "[ CartTargetKinematicFilter ] Scaled Joint Vel (after joint saturation) "  << cmd_qd.transpose() );
    m_target_p  = m_chain->getTransformation( cmd_q );
    m_target_v  = m_chain->getJacobian( cmd_q ) * cmd_qd;
    //ROS_DEBUG_STREAM_THROTTLE(2,  "[ CartTargetKinematicFilter ] Scaled  Cart Vel (after joint saturation) "  << m_target_v.transpose() );

    // ==============================

//    ROS_DEBUG_STREAM_THROTTLE(2,  "Last Target p: " << m_last_target_p.matrix() );
//    ROS_DEBUG_STREAM_THROTTLE(2,  "New  Target p: " << m_target_p.matrix() );
//    ROS_DEBUG_STREAM_THROTTLE(2,  "Last Target v: " << m_last_target_v.transpose() );
//    ROS_DEBUG_STREAM_THROTTLE(2,  "New  Target v: " << m_target_v.transpose() );
//    ROS_DEBUG_STREAM_THROTTLE(2,  "Actual Pos   : " << actual_p.matrix() );

  }
  catch(...)
  {
    ROS_WARN(" something wrong in JointTargetFilter::update" );
    cmd_q  = m_last_target_q;
    cmd_qd.setZero();
  }

}


}
}




