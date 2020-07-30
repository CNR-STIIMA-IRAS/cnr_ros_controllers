#include <eigen_conversions/eigen_msg.h>
#include <cnr_cart_teleop_controller/cnr_cart_teleop_controller.h>
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/algorithm/string.hpp>

PLUGINLIB_EXPORT_CLASS(cnr::control::CartTeleopController, controller_interface::ControllerBase);


namespace cnr
{
namespace control
{

bool CartTeleopController::doInit()
{
  CNR_TRACE_START(*m_logger);

  inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::doInit();


  m_twist_in_b        = Eigen::Vector6d::Zero();
  m_last_twist_in_b   = Eigen::Vector6d::Zero();

  m_listener.reset( new tf::TransformListener( getControllerNh() ) );

  std::string setpoint_topic_name = getControllerNamespace()+"/target_cart_teleop";
  add_subscriber("target_sub", setpoint_topic_name, 1,  &cnr::control::CartTeleopController::setTargetCallback, this );

  CNR_RETURN_TRUE(*m_logger);
}

void CartTeleopController::setTargetCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  try
  {
    CNR_DEBUG_THROTTLE(*m_logger, 2, ">>>>>>>>>> TWIST TARGET TARGET RECEIVED!");

    Eigen::Vector6d twist = Eigen::Vector6d::Zero( );
    tf::twistMsgToEigen(*msg, twist);

    if(std::isnan(twist.norm()))
    {
      CNR_WARN_THROTTLE(*m_logger, 2, "SAFETY CHECK - Received a Twist with nan values... superimposed to zero!");
      twist = Eigen::Vector6d::Zero();
    }

    CNR_DEBUG_THROTTLE(*m_logger, 2,"Reference Twist {" << msg->header.frame_id << "}     : " << twist.transpose() );

    std::string frame_id = boost::to_lower_copy( msg->header.frame_id);

    if ( frame_id == "tool" )
    {
      m_twist_in_b = rosdyn::spatialRotation( twist, m_Tbt.rotation());
    }
    else if ( frame_id == "base" )
    {
      m_twist_in_b = twist;
    }
    else
    {
      tf::StampedTransform TF_T_bf;
      CNR_DEBUG_THROTTLE(*m_logger, 5,"listening to transform between "<<m_base_link<<" and "<<msg->header.frame_id);
      m_listener->waitForTransform ( m_base_link, msg->header.frame_id, ros::Time(0), ros::Duration ( 10.0 ) );
      m_listener->lookupTransform  ( m_base_link, msg->header.frame_id, ros::Time(0), TF_T_bf);
      Eigen::Affine3d T_bf;
      tf::transformTFToEigen(TF_T_bf, T_bf);

      m_twist_in_b = rosdyn::spatialRotation( twist, T_bf.rotation());
    }
    CNR_DEBUG_THROTTLE(*m_logger, 5,"Reference Twist {base}     : " << m_twist_in_b.transpose() );
  }
  catch(tf::TransformException& e)
  {
    CNR_WARN(*m_logger, "something wrong in Getting the data from tf");
    CNR_WARN(*m_logger, "Listening to transform between "<<m_base_link<<" and "<<msg->header.frame_id <<" failed" );
    m_twist_in_b = Eigen::Vector6d::Zero();
  }
  catch(std::exception& e)
  {
    CNR_WARN(*m_logger, "something wrong in Getting the data from tf");
    CNR_WARN(*m_logger, "Exception "<< e.what() );
    m_twist_in_b = Eigen::Vector6d::Zero();
  }
  catch(...)
  {
    CNR_WARN(*m_logger, "something wrong in Getting the data from tf");
    m_twist_in_b = Eigen::Vector6d::Zero();
  }
  CNR_DEBUG_THROTTLE(*m_logger, 2, "<<<<<<<<< TWIST TARGET TARGET RECEIVED!"  );
}

bool CartTeleopController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);

  inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::doStarting(time);

  m_target_p      = m_Tbt;
  m_last_target_p = m_target_p;

  m_target_v      = m_J *qd();
  m_last_target_v = m_target_v;

  m_last_target_v.setZero();
  m_target_v.setZero();

  CNR_RETURN_TRUE(*m_logger);
}

bool CartTeleopController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::doStopping(time);
  CNR_RETURN_TRUE(*m_logger);
}

bool CartTeleopController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START(*m_logger);
  m_last_target_p = m_target_p;
  m_last_target_v = m_target_v;
  double dt = period.toSec() >= m_sampling_period ? period.toSec() : m_sampling_period;

  Eigen::Vector6d         cmd_twist = Eigen::Vector6d::Zero();
  geometry_msgs::TwistPtr cmd_twist_msgs( new geometry_msgs::Twist() );
  try
  {
    //==============================
    Eigen::VectorXd distance;
    Eigen::Vector6d nominal_v; nominal_v.setZero();
    if( m_priority == JointCommandController::Q_PRIORITY )
    {
      rosdyn::getFrameDistance(m_target_p,m_last_target_p, distance);

      nominal_v = distance / dt;
    }
    else if( m_priority == JointCommandController::QD_PRIORITY )
    {
      nominal_v = m_twist_in_b;
      if(std::isnan(nominal_v.norm()))
      {
        nominal_v.setZero();
      }
    }
    //==============================


    // ==============================
    if( nominal_v.norm() > 0 )
    {
      cmd_twist = nominal_v.block(0,0,3,1).norm() > m_max_cart_lin_vel ? nominal_v * m_max_cart_lin_vel / ( nominal_v.block(0,0,3,1).norm() ) : nominal_v;
      cmd_twist = cmd_twist.block(3,0,3,1).norm() > m_max_cart_ang_vel ? cmd_twist * m_max_cart_ang_vel / ( cmd_twist.block(3,0,3,1).norm() ) : cmd_twist;
    }

    Eigen::Vector6d dtwist = (cmd_twist - m_last_target_v)/dt;
    if(std::isnan(dtwist.norm()))
    {
      dtwist = Eigen::Vector6d::Zero();
    }

    if( dtwist.norm() > 0 )
    {
      dtwist *= dtwist.block(0,0,3,1).norm() > m_max_cart_lin_acc ? m_max_cart_lin_acc/dtwist.block(0,0,3,1).norm() : 1.0;
      dtwist *= dtwist.block(3,0,3,1).norm() > m_max_cart_ang_acc ? m_max_cart_ang_acc/dtwist.block(3,0,3,1).norm() : 1.0;
    }

    cmd_twist = m_last_target_v + dtwist * dt;
    geometry_msgs::Twist cmd_twist_msgs_tmp;
    tf::twistEigenToMsg(cmd_twist, cmd_twist_msgs_tmp);
    *cmd_twist_msgs = cmd_twist_msgs_tmp;
    InverseKinematicsQpPosVelEffController::targetTwistCallback(cmd_twist_msgs);

    m_target_p  = m_chain->getTransformation( m_target.q );
    m_target_v  = m_chain->getJacobian( m_target.q ) * m_target.qd;

  }
  catch(...)
  {
    CNR_WARN(*m_logger,"something wrong in update" );
    cmd_twist.setZero();
    geometry_msgs::Twist cmd_twist_msgs_tmp;
    tf::twistEigenToMsg(cmd_twist, cmd_twist_msgs_tmp);
    *cmd_twist_msgs = cmd_twist_msgs_tmp;
    InverseKinematicsQpPosVelEffController::targetTwistCallback(cmd_twist_msgs);
  }

  CNR_RETURN_BOOL(*m_logger, InverseKinematicsQpPosVelEffController::doUpdate(time, period));
}

}
}
