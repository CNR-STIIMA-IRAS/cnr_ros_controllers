#include <boost/algorithm/string.hpp>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <pluginlib/class_list_macros.h>
#include <cnr_logger/cnr_logger_macros.h>

#include <cnr_cartesian_position_controller/cnr_cartesian_position_controller.h>


PLUGINLIB_EXPORT_CLASS(cnr::control::CartesianPositionController, controller_interface::ControllerBase)

namespace cnr
{
namespace control
{


/**
 * @brief CartesianPositionController::CartesianPositionController
 */
inline CartesianPositionController::CartesianPositionController()
{
}

/**
 * @brief CartesianPositionController::doInit
 * @return
 */
inline bool CartesianPositionController::doInit()
{
  //INIT PUB/SUB
  std::string setpoint_topic_name;
  this->setKinUpdatePeriod(this->m_sampling_period); // superimposing the fkin_update_period,
                                                     // we can then use chainCommand() sync and updated

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

  if (!this->getControllerNh().getParam("click_gain",m_clik_gain))
  {
    CNR_INFO(this->logger(),this->getControllerNamespace()<<"/click_gain not defined, using 5.0");
    m_clik_gain=5.0;
  }

  m_as.reset(new actionlib::ActionServer<cnr_cartesian_position_controller::RelativeMoveAction>(this->getControllerNh(), "relative_move",
                                                                      boost::bind(&CartesianPositionController::actionGoalCallback,    this,  _1),
                                                                      boost::bind(&CartesianPositionController::actionCancelCallback,  this,  _1),
                                                                      false));
  m_as->start();

  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianPositionController::doStarting
 * @param time
 */
inline bool CartesianPositionController::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  this->setCommandVelocity(0.0*this->getVelocity()); //not needed, already superimposed in enterStarting()
  this->setCommandPosition(this->getPosition());

  mtx_.lock();
  rosdyn::VectorXd q = this->getPosition();
  T_base_t_=this->chainNonConst().getTransformation(q);
  T_base_target_=T_base_t_;
  mtx_.unlock();
  ROS_FATAL_STREAM_THROTTLE(0.5,"T_base_target = "<<T_base_target_.matrix());
  ROS_FATAL_STREAM_THROTTLE(0.5,"T_base_t = "<<T_base_t_.matrix());

  last_twist_of_in_b_ = Eigen::Vector6d::Zero();
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianPositionController::stopping
 * @param time
 */
inline bool CartesianPositionController::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");

  m_stop_thread=true;
  if (m_as_thread.joinable())
  {
    m_as_thread.join();
  }
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief CartesianPositionController::doUpdate
 * @param time
 * @param period
 * @return
 */
inline bool CartesianPositionController::doUpdate(const ros::Time& /*time*/, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;


  mtx_.lock();
  rosdyn::VectorXd old_vel_sp = this->getCommandVelocity();
  rosdyn::VectorXd pos_sp = this->getCommandPosition();
//  rosdyn::VectorXd q = this->getPosition();
  T_base_t_=this->chainNonConst().getTransformation(pos_sp);
  Eigen::Affine3d T_base_target=T_base_target_;
  mtx_.unlock();

  Eigen::Vector6d distance_in_base;
  rosdyn::getFrameDistance(T_base_target,T_base_t_,distance_in_base);

  Eigen::Vector6d twist_of_t_in_b = m_clik_gain*distance_in_base;

  if (twist_of_t_in_b.block(0,0,3,1).norm() > max_cart_lin_vel_)
    twist_of_t_in_b *= max_cart_lin_vel_/twist_of_t_in_b.norm();

  if (twist_of_t_in_b.block(0,0,3,1).norm() > target_velocity_)
    twist_of_t_in_b *= target_velocity_/twist_of_t_in_b.norm();


  if (twist_of_t_in_b.block(3,0,3,1).norm()>max_cart_ang_vel_)
    twist_of_t_in_b*=max_cart_ang_vel_/twist_of_t_in_b.norm();

  Eigen::Vector6d Dtwist_of_t_in_b;
  if (period.toSec()>0.0)
  {
    Dtwist_of_t_in_b = (twist_of_t_in_b-last_twist_of_in_b_)/period.toSec();
    double scaling=1.0;
    if (Dtwist_of_t_in_b.block(0,0,3,1).norm()>max_cart_lin_acc_)
      scaling=max_cart_lin_acc_/Dtwist_of_t_in_b.norm();

    if (Dtwist_of_t_in_b.block(3,0,3,1).norm()>max_cart_ang_acc_)
      scaling=std::min(scaling,max_cart_ang_acc_/Dtwist_of_t_in_b.norm());

    Dtwist_of_t_in_b*=scaling;
    twist_of_t_in_b=last_twist_of_in_b_+Dtwist_of_t_in_b*period.toSec();
  }
  else
  {
    twist_of_t_in_b = Eigen::Vector6d::Zero( );
    last_twist_of_in_b_ = Eigen::Vector6d::Zero( );
  }

  Eigen::Matrix6Xd J_of_t_in_b;
  J_of_t_in_b=this->chainCommand().toolJacobian();  // CHECK IF CORRECT

  Eigen::FullPivLU<Eigen::MatrixXd> pinv_J(J_of_t_in_b);

  pinv_J.setThreshold ( 1e-2 );


  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_of_t_in_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
  auto sv = svd.singularValues();
  CNR_WARN_COND_THROTTLE(this->logger(),
                          (sv(sv.rows()-1)==0) || (sv(0)/sv(sv.rows()-1) > 1e2), 2, "SINGULARITY POINT" );

  if(pinv_J.rank()<6)
  {
    CNR_WARN_THROTTLE(this->logger(),2,"rank: "<<pinv_J.rank()<<"\nJacobian\n"<<J_of_t_in_b);
  }

  rosdyn::VectorXd vel_sp = svd.solve(twist_of_t_in_b);

  if(rosdyn::saturateSpeed(this->chainNonConst(),vel_sp,old_vel_sp,
                           this->getCommandPosition(),period.toSec(), 1.0, true, &report)) // CHECK!
  {
    CNR_DEBUG_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }

  Eigen::Vector6d twist_of_t_in_bcommand=J_of_t_in_b*vel_sp;
  Eigen::Vector6d versor=twist_of_t_in_b.normalized();
  Eigen::Vector6d parallel_twist=twist_of_t_in_bcommand.dot(versor)*versor;
  Eigen::Vector6d perpendicular_twist=twist_of_t_in_bcommand -parallel_twist;

  if (perpendicular_twist.norm()>1e-6)
  {
    vel_sp*=1e-6/perpendicular_twist.norm();
    CNR_WARN_THROTTLE(this->logger(),1,"saturating velocity, direction error (perpendicular norm = " << perpendicular_twist.norm() <<  ") due to singularity and joint limits");
    CNR_DEBUG_THROTTLE(this->logger(),1,
                      "twist_of_t_in_b         = " << twist_of_t_in_b.transpose() << std::endl <<
                      "twist_of_t_in_bcommand = " << twist_of_t_in_bcommand.transpose() << std::endl <<
                      "parallel_twist velocity = " << parallel_twist.transpose() << std::endl <<
                      "perpedicular velocity   = " << perpendicular_twist.transpose()
                      );
  }
  last_twist_of_in_b_=J_of_t_in_b*vel_sp;

  if(rosdyn::saturateSpeed(this->chainNonConst(),vel_sp,old_vel_sp,
                              this->getCommandPosition(),period.toSec(), 1.0, true, &report)) // CHECK!
  {
    CNR_DEBUG_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }


  pos_sp = this->getCommandPosition() + vel_sp * period.toSec();

  if(rosdyn::saturatePosition(this->chainNonConst(),pos_sp, &report))
  {
    CNR_DEBUG_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
  }

  last_twist_of_in_b_=J_of_t_in_b*vel_sp;
  this->setCommandPosition( pos_sp );
  this->setCommandVelocity( vel_sp );

  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

/**
 * @brief CartesianPositionController::actionGoalCallback
 * @param gh
 */
void CartesianPositionController::actionGoalCallback(actionlib::ActionServer< cnr_cartesian_position_controller::RelativeMoveAction>::GoalHandle gh)
{
  try
  {
    auto goal = gh.getGoal();

    std::shared_ptr<actionlib::ActionServer<cnr_cartesian_position_controller::RelativeMoveAction>::GoalHandle> current_gh;

    current_gh.reset(new actionlib::ActionServer<cnr_cartesian_position_controller::RelativeMoveAction>::GoalHandle(gh));
    m_gh = current_gh;

    mtx_.lock();
    Eigen::Affine3d T_b_actual=T_base_t_;
    mtx_.unlock();

    Eigen::Affine3d T_t_target;
    tf::poseMsgToEigen(goal->relative_pose.pose,T_t_target);
    if (goal->relative_pose.header.frame_id=="TOOL")
    {
      tf::poseMsgToEigen(goal->relative_pose.pose,T_t_target);
    }
    else if (goal->relative_pose.header.frame_id=="BASE")
    {
      Eigen::Affine3d T_t_target_in_base;
      tf::poseMsgToEigen(goal->relative_pose.pose,T_t_target_in_base);
      Eigen::AngleAxisd aa_in_b(T_t_target_in_base.linear());
      Eigen::Vector3d axis_in_b=aa_in_b.axis();
      double angle=aa_in_b.angle();
      Eigen::Vector3d axis_in_actual=T_b_actual.linear().inverse()*axis_in_b;

      Eigen::AngleAxisd aa_in_actual(angle,axis_in_actual);
      T_t_target=aa_in_actual;

      Eigen::Vector3d p_target_in_b=T_t_target_in_base.translation();
      Eigen::Vector3d p_target_in_actual=T_b_actual.linear().inverse()*p_target_in_b;
      T_t_target.translation()=p_target_in_actual;
      ROS_FATAL_STREAM("T_t_target_in_base = " << T_t_target_in_base.matrix());
      ROS_FATAL_STREAM("T_t_target = " << T_t_target.matrix());

    }
    else
    {
      ROS_FATAL("not supported yet");
      T_t_target.setIdentity();
      cnr_cartesian_position_controller::RelativeMoveResult result;
      result.error_code   = -1;
      result.error_string = "unsupported feature";
      m_gh->setRejected(result);
      return;
    }
    target_velocity_=goal->target_velocity;
    if (target_velocity_<=0)
    {
      ROS_FATAL("target velocity should be positive");
      T_t_target.setIdentity();
      cnr_cartesian_position_controller::RelativeMoveResult result;
      result.error_code   = -1;
      result.error_string = "target velocity should be positive";
      m_gh->setRejected(result);
      return;
    }

    CNR_INFO(this->logger(),"[ "<<this->getControllerNamespace()<<" ] New Goal Received, action start!");
    m_gh->setAccepted();

    mtx_.lock();
    T_base_target_=T_base_t_*T_t_target;
    mtx_.unlock();

    if (m_as_thread.joinable())
    {
      m_as_thread.join();
    }

    m_as_thread    = std::thread(&CartesianPositionController::actionThreadFunction,this);
  }
  catch( std::exception& e )
  {
    ROS_ERROR_STREAM("Exception. what: " << e.what() );
    cnr_cartesian_position_controller::RelativeMoveResult result;
    result.error_code   = -1;
    result.error_string = std::string("exception: ")+e.what();
    m_gh->setAborted(result);
  }
  catch( ... )
  {
    ROS_ERROR_STREAM("Generalized Exception.");
    cnr_cartesian_position_controller::RelativeMoveResult result;
    result.error_code   = -1;
    result.error_string = "goal exception";
    m_gh->setAborted(result);
  }
}

void CartesianPositionController::actionCancelCallback(actionlib::ActionServer< cnr_cartesian_position_controller::RelativeMoveAction >::GoalHandle /*gh*/)
{
  ROS_DEBUG("[ %s ] Triggered the Cancel of the Action",  this->getControllerNamespace().c_str());
  if (m_gh)
  {
    m_gh->setCanceled();
    m_stop_thread = true;
    if (m_as_thread.joinable())
    {
      m_as_thread.join();
    }
    m_gh.reset();
  }
  else
  {
    ROS_WARN("[ %s ] Triggered the Cancel of the Action but none Goal is active.",  this->getControllerNamespace().c_str());
  }
  ROS_DEBUG("[ %s ] Action Succesfully Cancelled",  this->getControllerNamespace().c_str());
}

void CartesianPositionController::actionThreadFunction()
{
  ROS_DEBUG("[ %s ] START ACTION GOAL LOOP",  this->getControllerNamespace().c_str());
  ros::WallRate lp(100);

  ros::Time leaving_timer;

  while (this->getControllerNh().ok())
  {
    lp.sleep();
    if (!m_gh)
    {
      ROS_ERROR("[ %s ] Goal handle is not initialized",  this->getControllerNamespace().c_str());
      break;
    }
    cnr_cartesian_position_controller::RelativeMoveFeedback fb;
    cnr_cartesian_position_controller::RelativeMoveResult result;
    mtx_.lock();
    Eigen::Affine3d T_b_actual=T_base_t_;
    Eigen::Affine3d T_b_target=T_base_target_;
    mtx_.unlock();

    Eigen::Vector6d distance;
    rosdyn::getFrameDistance(T_b_target,T_b_actual,distance);
    if (distance.head(3).norm()<m_lin_tolerance && distance.tail(3).norm()<m_ang_tolerance)
    {
      result.error_code   = 0;
      result.error_string = "finished";
      m_gh->setSucceeded(result);
      ROS_FATAL("FINITO");
      break;
    }

    if( m_stop_thread )
    {
      ROS_ERROR("[ %s ] Triggered an external stop. Break the action loop.",  this->getControllerNamespace().c_str());
      result.error_code   = -1;
      result.error_string = ".............";
      m_gh->setAborted(result);
      break;
    }

    m_gh->publishFeedback(fb);
  }
  m_gh.reset();
  ROS_FATAL("exit thread");
}


}
}
