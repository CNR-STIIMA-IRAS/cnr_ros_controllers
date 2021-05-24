#include <boost/algorithm/string.hpp>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <state_space_filters/filtered_values.h>
#include <eigen_matrix_utils/overloads.h>
#include <cnr_cartesian_velocity_controller/cnr_cartesian_velocity_controller.h>

#include <pluginlib/class_list_macros.h>

namespace eu = eigen_utils;
namespace ect = eigen_control_toolbox;

PLUGINLIB_EXPORT_CLASS(cnr::control::CartesianVelocityController, controller_interface::ControllerBase)

namespace cnr
{
namespace control
{


/**
 * @brief CartesianVelocityController::CartesianVelocityController
 */
inline CartesianVelocityController::CartesianVelocityController()
{
}

/**
 * @brief CartesianVelocityController::doInit
 * @return
 */
inline bool CartesianVelocityController::doInit()
{
  //INIT PUB/SUB
  std::string setpoint_topic_name;
  setpoint_topic_name = this->getControllerNamespace() + "/target_joint_teleop";

  this->setKinUpdatePeriod(this->m_sampling_period); // superimposing the fkin_update_period, 
                                                     // we can then use chainCommand() sync and updated

  if(!this->getControllerNh().getParam("target_twist_topic",setpoint_topic_name))
      CNR_WARN(this->logger(),"target_twist_topic not set. Default value superimposed.");

  this->template add_subscriber<geometry_msgs::TwistStamped>(
        setpoint_topic_name,5,boost::bind(&CartesianVelocityController::callback,this,_1), false);

  this->setPriority(this->QD_PRIORITY);

  this->setCommandVelocity(0.0*this->getVelocity()); //not needed, already superimposed in enterStarting()
  this->setCommandPosition(this->getPosition());

  if (!this->getControllerNh().getParam("max_cartesian_linear_speed",max_cart_lin_vel_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/max_cartesian_linear_speed not defined, using 0.25 m/s");
    max_cart_lin_vel_=0.25;
  }

  if (!this->getControllerNh().getParam("max_cartesian_linear_acceleration",max_cart_lin_acc_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/max_cartesian_linear_acceleration not defined, using 0.75 m/s^2");
    max_cart_lin_acc_=0.75;
  }

  if (!this->getControllerNh().getParam("max_cartesian_angular_speed",max_cart_ang_vel_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/max_cartesian_angular_speed not defined, using 0.5 rad/s");
    max_cart_ang_vel_=0.5;
  }

  if (!this->getControllerNh().getParam("max_cartesian_angular_acceleration",max_cart_ang_acc_))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/max_cartesian_angular_acceleration not defined, using 1.5 rad/s^2");
    max_cart_ang_acc_=1.5;
  }

  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianVelocityController::doStarting
 * @param time
 */
inline bool CartesianVelocityController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  this->setCommandVelocity(0.0*this->getVelocity()); //not needed, already superimposed in enterStarting()
  this->setCommandPosition(this->getPosition());

  last_twist_of_in_b_ = Eigen::Vector6d::Zero();
  twist_of_t_in_b_ = Eigen::Vector6d::Zero();
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianVelocityController::stopping
 * @param time
 */
inline bool CartesianVelocityController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianVelocityController::doUpdate
 * @param time
 * @param period
 * @return
 */
inline bool CartesianVelocityController::doUpdate(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;

  m_mtx.lock();
  Eigen::Vector6d twist_of_t_in_b = twist_of_t_in_b_;
  m_mtx.unlock();

  if (twist_of_t_in_b.block(0,0,3,1).norm() > max_cart_lin_vel_)
    twist_of_t_in_b *= max_cart_lin_vel_/twist_of_t_in_b.norm();

  if (twist_of_t_in_b.block(3,0,3,1).norm()>max_cart_ang_vel_)
    twist_of_t_in_b*=max_cart_ang_vel_/twist_of_t_in_b.norm();

  Eigen::Vector6d Dtwist_of_t_in_b;
  if (period.toSec()>0.0)
  {
    Dtwist_of_t_in_b = (twist_of_t_in_b-last_twist_of_in_b_)/period.toSec();
    
    if (Dtwist_of_t_in_b.block(0,0,3,1).norm()>max_cart_lin_acc_)
      Dtwist_of_t_in_b*=max_cart_lin_acc_/Dtwist_of_t_in_b.norm();
    
    if (Dtwist_of_t_in_b.block(3,0,3,1).norm()>max_cart_ang_acc_)
      Dtwist_of_t_in_b*=max_cart_ang_acc_/Dtwist_of_t_in_b.norm();
    
    twist_of_t_in_b=last_twist_of_in_b_+Dtwist_of_t_in_b*period.toSec();
    last_twist_of_in_b_=twist_of_t_in_b;
  }
  else
  {
    twist_of_t_in_b = Eigen::Vector6d::Zero( );
    last_twist_of_in_b_ = Eigen::Vector6d::Zero( );
  }

  rosdyn::VectorXd vel_sp = this->getCommandVelocity();
  rosdyn::VectorXd pos_sp = this->getCommandPosition();
  Eigen::Matrix6Xd J_of_t_in_b;

  J_of_t_in_b=this->chainCommand().toolJacobian();  // CHECK IF CORRECT

  Eigen::FullPivLU<Eigen::MatrixXd> pinv_J(J_of_t_in_b);

  pinv_J.setThreshold ( 1e-2 );
  if(pinv_J.rank()<6)
  {
    CNR_FATAL(this->logger(),"rank: "<<pinv_J.rank()<<"\nJacobian\n"<<J_of_t_in_b);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_of_t_in_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
  CNR_WARN_COND_THROTTLE(this->logger(),
    (svd.singularValues()(svd.cols()-1)==0) || (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2),
    2, "SINGULARITY POINT" );

  vel_sp = svd.solve(twist_of_t_in_b);
  if(rosdyn::saturateSpeed(this->chainNonConst(),vel_sp,vel_sp,
                              this->getCommandPosition(),period.toSec(), 1.0, true, &report)) // CHECK!
  {
    CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }

  pos_sp = this->getCommandPosition() + vel_sp * period.toSec();
  
  if(rosdyn::saturatePosition(this->chainNonConst(),pos_sp, &report))
  {
    CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }

  this->setCommandPosition( pos_sp );
  this->setCommandVelocity( vel_sp );

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

/**
 * @brief CartesianVelocityController::callback
 * @param msg
 */
inline void CartesianVelocityController::callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  Eigen::Vector6d twist_of_t_in_b = Eigen::Vector6d::Zero( );
  std::string base_link = this->chain().getLinksName().front();

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
      CNR_WARN_THROTTLE(this->logger(), 2, "[ " << this->getControllerNamespace() 
                          <<" ] SAFETY CHECK - Received a Twist with nan values... superimposed to zero!" );
      twist = Eigen::Vector6d::Zero();
    }

    CNR_WARN_THROTTLE( this->logger(), 2, "[ " << this->getControllerNamespace()
                          <<" ] Reference Twist {" << msg->header.frame_id << "}     : " << twist.transpose() );

    std::string frame_id = boost::to_lower_copy( msg->header.frame_id);

    Eigen::Affine3d Tbt = this->chainCommand().toolPose();

    if ( frame_id == "tool" )
    {
      twist_of_t_in_b = rosdyn::spatialRotation( twist, Tbt.rotation());
    }
    else if ( frame_id == "base" )
    {
      twist_of_t_in_b = twist;
    }
    else
    {
      tf::StampedTransform TF_T_bf;
      CNR_DEBUG_THROTTLE(this->logger(), 2, "[ "
                            << this->getControllerNamespace() << " ] listening to transform between "<<base_link<<" and "
                                <<msg->header.frame_id);
      listener_.waitForTransform ( base_link, msg->header.frame_id, ros::Time(0), ros::Duration ( 10.0 ) );
      listener_.lookupTransform  ( base_link, msg->header.frame_id, ros::Time(0), TF_T_bf);
      Eigen::Affine3d T_bf;
      tf::transformTFToEigen(TF_T_bf, T_bf);

      twist_of_t_in_b = rosdyn::spatialRotation( twist, T_bf.rotation());
    }
    CNR_DEBUG_THROTTLE( this->logger(), 2, "[ " << this->getControllerNh().getNamespace() 
                          << " ] Reference Twist {base}     : " << twist_of_t_in_b_.transpose() );
  }
  catch(tf::TransformException& e)
  {
    CNR_WARN(this->logger(), "[ " << this->getControllerNamespace() << " ] Listening to transform between "<<base_link
                <<" and "<<msg->header.frame_id <<" failed" );
    twist_of_t_in_b = Eigen::Vector6d::Zero();
  }
  catch(std::exception& e)
  {
    CNR_WARN(this->logger(), "[ " << this->getControllerNamespace() << " ]Exception "<< e.what() );
    twist_of_t_in_b = Eigen::Vector6d::Zero();
  }
  catch(...)
  {
    CNR_WARN(this->logger(), "[ " << this->getControllerNamespace() << " ] unhandled excpetion..");
    twist_of_t_in_b = Eigen::Vector6d::Zero();
  }
  CNR_DEBUG_THROTTLE(this->logger(), 2, "[ " << this->getControllerNamespace() << " ] <<<<<<<<< TWIST TARGET TARGET RECEIVED!"  );
  std::lock_guard<std::mutex> lock(m_mtx);
  twist_of_t_in_b_=twist_of_t_in_b;
  return;
}

}
}
