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

    this->template add_subscriber<geometry_msgs::TwistStamped>(
    setpoint_topic_name,5,boost::bind(&CartesianTeleopController::callback,this,_1), false);

    this->setPriority(this->QD_PRIORITY);


    m_vel_sp = 0.0*this->getVelocity();
    m_pos_sp = this->getPosition();

    if (!this->getControllerNh().getParam("max_cartesian_linear_speed",m_max_cart_lin_vel))
    {
      ROS_INFO("%s/max_cartesian_linear_speed not defined, using 0.25 m/s",this->getControllerNh().getNamespace().c_str());
      m_max_cart_lin_vel=0.25;
    }

    if (!this->getControllerNh().getParam("max_cartesian_linear_acceleration",m_max_cart_lin_acc))
    {
      ROS_INFO("%s/max_cartesian_linear_acceleration not defined, using 0.75 m/s^2",this->getControllerNh().getNamespace().c_str());
      m_max_cart_lin_acc=0.75;
    }

    if (!this->getControllerNh().getParam("max_cartesian_angular_speed",m_max_cart_ang_vel))
    {
      ROS_INFO("%s/max_cartesian_angular_speed not defined, using 0.5 rad/s",this->getControllerNh().getNamespace().c_str());
      m_max_cart_ang_vel=0.5;
    }

    if (!this->getControllerNh().getParam("max_cartesian_angular_acceleration",m_max_cart_ang_acc))
    {
      ROS_INFO("%s/max_cartesian_angular_acceleration not defined, using 1.5 rad/s^2",this->getControllerNh().getNamespace().c_str());
      m_max_cart_ang_acc=1.5;
    }



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

    m_last_twist_of_in_b = Eigen::Vector6d::Zero();
    m_twist_of_t_in_b = Eigen::Vector6d::Zero();
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
  inline bool CartesianTeleopController::doUpdate(const ros::Time& time, const ros::Duration& period)
  {
    CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
    std::stringstream report;

    m_mtx.lock();
    Eigen::Vector6d twist_of_t_in_b = m_twist_of_t_in_b;
    m_mtx.unlock();

    if (twist_of_t_in_b.block(0,0,3,1).norm()>m_max_cart_lin_vel)
      twist_of_t_in_b*=m_max_cart_lin_vel/twist_of_t_in_b.norm();
    if (twist_of_t_in_b.block(3,0,3,1).norm()>m_max_cart_ang_vel)
      twist_of_t_in_b*=m_max_cart_ang_vel/twist_of_t_in_b.norm();


    Eigen::Vector6d Dtwist_of_t_in_b;
    if (period.toSec()>0.0)
    {
      Dtwist_of_t_in_b = (twist_of_t_in_b-m_last_twist_of_in_b)/period.toSec();
      if (Dtwist_of_t_in_b.block(0,0,3,1).norm()>m_max_cart_lin_acc)
        Dtwist_of_t_in_b*=m_max_cart_lin_acc/Dtwist_of_t_in_b.norm();
      if (Dtwist_of_t_in_b.block(3,0,3,1).norm()>m_max_cart_ang_acc)
        Dtwist_of_t_in_b*=m_max_cart_ang_acc/Dtwist_of_t_in_b.norm();
      twist_of_t_in_b=m_last_twist_of_in_b+Dtwist_of_t_in_b*period.toSec();
      m_last_twist_of_in_b=twist_of_t_in_b;
    }
    else
    {
      twist_of_t_in_b = Eigen::Vector6d::Zero( );
      m_last_twist_of_in_b = Eigen::Vector6d::Zero( );
    }

    rosdyn::VectorXd vel_sp = m_vel_sp;
    rosdyn::VectorXd pos_sp = m_pos_sp;


    Eigen::Matrix6Xd J_of_t_in_b;

    J_of_t_in_b=this->chainNonConst().getJacobian(pos_sp);  // CHECK IF CORRECT

    Eigen::FullPivLU<Eigen::MatrixXd> pinv_J(J_of_t_in_b);

    pinv_J.setThreshold(1e-2);
    vel_sp = pinv_J.solve(twist_of_t_in_b);
    if (pinv_J.rank()<6)
    {
      Eigen::MatrixXd null=pinv_J.kernel();
      CNR_WARN_THROTTLE(this->logger(),2,"Singolarity point!");

      for (int iC=0;iC<null.cols();iC++)
      {
        Eigen::VectorXd null_versor=null.col(iC);
        null_versor.normalize();
        vel_sp=vel_sp-(vel_sp.dot(null_versor))*null_versor;
      }
    }

    if(rosdyn::saturateSpeed(this->chainNonConst(),vel_sp,m_vel_sp,m_pos_sp,period.toSec(), 1.0, 1.0, &report)) // CHECK!
    {
      CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
    }


    m_pos_sp = m_pos_sp + vel_sp* period.toSec();
    pos_sp   = m_pos_sp;

    if(rosdyn::saturatePosition(this->chainNonConst(),pos_sp, &report))
    {
      CNR_WARN_THROTTLE(this->logger(), 2.0, "\n" << report.str() );
    }
    m_pos_sp  = pos_sp;

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
    Eigen::Vector6d twist_of_t_in_b = Eigen::Vector6d::Zero( );

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
        twist_of_t_in_b = rosdyn::spatialRotation( twist, this->chainCommand().toolPose().rotation());
      }
      else if ( frame_id == "base" )
      {
        twist_of_t_in_b = twist;
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

        twist_of_t_in_b = rosdyn::spatialRotation( twist, T_bf.rotation());
      }
      CNR_DEBUG_THROTTLE( this->logger(), 2, "[ " << this->getControllerNh().getNamespace() << " ] Reference Twist {base}     : " << m_twist_of_t_in_b.transpose() );
    }
    catch(tf::TransformException& e)
    {
      CNR_WARN_THROTTLE(this->logger(),2,"[ " << this->getControllerNh().getNamespace() << " ] Listening to transform between "<<this->chain().getLinksName().front()<<" and "<<msg->header.frame_id <<" failed" );
      twist_of_t_in_b = Eigen::Vector6d::Zero();
    }
    catch(std::exception& e)
    {
      CNR_WARN(this->logger(),"[ " << this->getControllerNh().getNamespace() << " ] Exception "<< e.what() );
      twist_of_t_in_b = Eigen::Vector6d::Zero();
    }
    catch(...)
    {
      CNR_WARN(this->logger(),"something wrong in Target Callback");
      twist_of_t_in_b = Eigen::Vector6d::Zero();
    }
    CNR_TRACE_THROTTLE(this->logger(),2, "[ " <<  this->getControllerNh().getNamespace() << " ] <<<<<<<<< TWIST TARGET TARGET RECEIVED!"  );

    std::lock_guard<std::mutex> lock(m_mtx);
    m_twist_of_t_in_b=twist_of_t_in_b;
    return;
  }


  }
}




