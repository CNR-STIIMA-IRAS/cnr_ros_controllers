#include <ros/ros.h>

#include <cnr_logger/cnr_logger.h>
#include <inverse_kinematics_qp_controller/inverse_kinematics_qp_controller.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(inverse_kinematics_qp::InverseKinematicsQpPosVelEffController, controller_interface::ControllerBase);


namespace std
{
inline std::string to_string( const std::vector<std::string>& vals )
{
  std::string ret = "< ";
  for( auto const & val : vals ) ret += val + ", ";
  ret += " >";
  return ret;
}
inline std::string to_string( const std::vector<double>& vals )
{
  std::string ret = "< ";
  for( auto const & val : vals ) ret += std::to_string(val) + ", ";
  ret += " >";
  return ret;
}
inline std::string to_string( const std::vector<int>& vals )
{
  std::string ret = "< ";
  for( auto const & val : vals ) ret += std::to_string(val) + ", ";
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

namespace inverse_kinematics_qp
{

/* ===============================================================================
 * ===============================================================================
 * ===============================================================================
 * ======= InverseKinematicsQpPosVelEffControllerPosVelEffController =============
 * ===============================================================================
 * ===============================================================================
 * =============================================================================== */

bool InverseKinematicsQpPosVelEffController::doInit()
{
  CNR_TRACE_START(*m_logger, "Initializing Inverse Kinematics QP controller '" + getControllerNh().getNamespace() + "'");

  bool cartesian_reference;
  if (!getControllerNh().getParam("cartesian_reference",cartesian_reference))
  {
    CNR_WARN(*m_logger, "cartesian_reference not defined, using joint reference");
    cartesian_reference=false;
  }
  if (cartesian_reference)
  {
    add_subscriber( "obstacle_subscriber",
      "/obstacle_in_b", 1, &inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::obstacleCallback, this);
    add_subscriber("target_pose_subscriber",
      "target_pose", 1, &inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::targetPoseCallback, this);
    add_subscriber("target_twist_subscriber",
      "target_twist",1, &inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::targetTwistCallback, this);
  }
  else
  {
    std::string joint_target;
    GET_AND_DEFAULT( getControllerNh(), "joint_target_topic", joint_target, "joint_target");

    add_subscriber("joint_reference_subscriber",
      joint_target,1,&inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::targetJointCallback,this);
  }

  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;

  std::vector<double> rest_config_std(nAx(), 0.0);
  Eigen::VectorXd rest_config(nAx());
  GET_AND_DEFAULT( getControllerNh(), "rest_config", rest_config_std, std::vector<double>(nAx(), 0.0) );

  for (size_t idx=0;idx<rest_config_std.size();idx++)
    rest_config(idx)=rest_config_std.at(idx);

  Eigen::VectorXd qmax(nAx());
  Eigen::VectorXd qmin(nAx());
  Eigen::VectorXd Dqmax(nAx());
  Eigen::VectorXd DDqmax(nAx());
  qmax.setConstant(3);
  qmin.setConstant(-3);
  Dqmax.setConstant(0.5);
  DDqmax.setConstant(4);

  double lambda_effort;
  double lambda_clik;
  double lambda_distance;
  double lambda_return;
  GET_AND_DEFAULT( getControllerNh(), "lambda_effort"  , lambda_effort  , 0.001);
  GET_AND_DEFAULT( getControllerNh(), "lambda_clik"    , lambda_clik    , 0.1  );
  GET_AND_DEFAULT( getControllerNh(), "lambda_distance", lambda_distance, 0.1  );
  GET_AND_DEFAULT( getControllerNh(), "lambda_return"  , lambda_return  , 0.008);

  std::vector<int> select_task_axis;
  GET_AND_DEFAULT( getControllerNh(), "select_task_axis" , select_task_axis, std::vector<int>(6,1) );

  if (select_task_axis.size()!=6 )
  {
    ROS_WARN("length of param 'select_task_axis' expected to be equal to 6 but is equal to %zu",select_task_axis.size());
    return false;
  }

  std::string secondary_task;
  GET_AND_DEFAULT( getControllerNh(), "secondary_task",secondary_task, "other" );

  if (secondary_task == "clearance")
  {
    std::string task2_frame;

    GET_AND_RETURN( getControllerNh(), "elbow_frame", task2_frame);

    rosdyn::ChainPtr chainTask2 = rosdyn::createChain(*m_model,m_base_link,task2_frame,grav);

    m_nAx2=chainTask2->getActiveJointsNumber();
    for (size_t idx=0;idx<m_nAx2;idx++)
    {
      m_joints_names_task2.push_back(jointNames().at(idx));
    }


    double clearance_threshold;
    GET_AND_DEFAULT( getControllerNh(), "clearance_threshold",clearance_threshold, 1);

    Eigen::VectorXd obstacle_in_b(3);
    obstacle_in_b(0)=1.5;
    obstacle_in_b(1)=-0.5;
    obstacle_in_b(2)=-0.5;
    m_obstacle_pose_in_b.linear().setIdentity();
    m_obstacle_pose_in_b.translation()=obstacle_in_b;
    m_obstacle_pose_vector.push_back(m_obstacle_pose_in_b);
    m_ik_solver.setClearanceOptions(chainTask2,m_nAx2,clearance_threshold,rest_config,m_obstacle_pose_in_b);
  }

  m_ik_solver.setAxisNumberTask1(nAx());
  m_ik_solver.setDynamicsChainTask1( m_chain );
  m_ik_solver.setConstraints(qmax,qmin,Dqmax,DDqmax);
  m_ik_solver.setSamplingPeriod(m_sampling_period);
  m_ik_solver.computeTaskSelectionMatrix(select_task_axis);
  m_ik_solver.setSecondaryTask(secondary_task);
  m_ik_solver.setWeigthFunction(lambda_distance,lambda_effort,lambda_return,lambda_clik);
  m_ik_solver.updateMatrices();

  GET_AND_DEFAULT( getControllerNh(), "max_cart_lin_vel", m_max_cart_lin_vel, 0.250);
  GET_AND_DEFAULT( getControllerNh(), "max_cart_lin_acc", m_max_cart_lin_acc, 0.250);
  GET_AND_DEFAULT( getControllerNh(), "max_cart_ang_vel", m_max_cart_ang_vel, 0.250);
  GET_AND_DEFAULT( getControllerNh(), "max_cart_ang_acc", m_max_cart_ang_acc, 0.250);

  CNR_RETURN_TRUE(*m_logger);
}

bool InverseKinematicsQpPosVelEffController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger, "Starting Inverse Kinematics QP controller '" + getControllerNh().getNamespace() + "'");

  Eigen::VectorXd qini(nAx());
  Eigen::VectorXd Dqini(nAx());
  for (size_t idx=0;idx<nAx();idx++)
  {
    qini(idx)= q(idx);
    Dqini(idx) = qd(idx);
  }

  m_ik_solver.setInitialState(qini,Dqini);
  m_next_vel.resize(nAx());
  m_next_vel.setZero();
  m_next_X=m_ik_solver.getPoseTask1();
  m_target_Dx.setZero();

  // seto obstacle e/o custom matrices

  CNR_RETURN_TRUE(*m_logger);
}

bool InverseKinematicsQpPosVelEffController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger, "Stopping Inverse Kinematics QP controller '" + getControllerNh().getNamespace()+"'");
  CNR_RETURN_TRUE(*m_logger);
}

bool InverseKinematicsQpPosVelEffController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START(*m_logger);
  try
  {
    // get m_target_Dx m_next_X
    // get obstacle vector from topic,
    m_ik_solver.computeClosestObstacle(m_obstacle_pose_vector,m_obstacle_pose_in_b);
    m_ik_solver.setObstaclePosition(m_obstacle_pose_in_b);
    // get custom matrices if necessary

    m_ik_solver.computedCostrainedSolution(m_target_Dx,
                                           m_next_X,
                                           m_ik_solver.getJointPosition(),
                                           m_ik_solver.getJointVelocity(),
                                           m_next_vel);
    m_ik_solver.updateState(m_next_vel);

    setPositionCommand( m_ik_solver.getJointPosition() );
    setVelocityCommand( m_ik_solver.getJointVelocity() );
    setEffortCommand  ( Eigen::Vector6d::Zero() );
  }
  catch (std::exception& e)
  {
    ROS_ERROR("something wrong: %s",e.what());
  }
  CNR_RETURN_TRUE(*m_logger);
}

void InverseKinematicsQpPosVelEffController::obstacleCallback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  if (msg->poses.size()>0)
  {
    if (m_base_link.compare(msg->header.frame_id))
      ROS_ERROR("Obstacle frame expected to be '%s' but received msg frame is '%s'. Message neglected.",m_base_link.c_str(),(msg->header.frame_id).c_str());
    else
    {
      m_obstacle_pose_vector.clear();
      for (size_t idx=0;idx<msg->poses.size();idx++)
      {
        Eigen::Affine3d obstacleEigen;
        tf::poseMsgToEigen(msg->poses.at(idx),obstacleEigen);
        m_obstacle_pose_vector.push_back(obstacleEigen);
      }
    }
  }
}

void InverseKinematicsQpPosVelEffController::targetPoseCallback(const geometry_msgs::PoseConstPtr& msg)
{
  tf::poseMsgToEigen(*msg,m_next_X);
}

void InverseKinematicsQpPosVelEffController::targetTwistCallback(const geometry_msgs::TwistConstPtr& msg)
{
  tf::twistMsgToEigen(*msg,m_target_Dx);
}

//void InverseKinematicsQpPosVelEffController::targetJointCallback(const sensor_msgs::JointStatePtr& msg)
//{

//}

void InverseKinematicsQpPosVelEffController::targetJointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  try
  {
    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(jointNames(),tmp_msg.name,tmp_msg.position,tmp_msg.velocity))
    {
      ROS_ERROR("joints not found");
      return;
    }

    rosdyn::ChainPtr chain=m_ik_solver.getDynamicsChainTask1();
    Eigen::VectorXd joint_pos_target(nAx());
    Eigen::VectorXd joint_vel_target(nAx());
    for (size_t idx=0;idx<nAx();idx++)
    {
      joint_pos_target(idx)=tmp_msg.position.at(idx);
      joint_vel_target(idx)=tmp_msg.velocity.at(idx);
    }
    m_next_X=chain->getTransformation(joint_pos_target);
    m_target_Dx=chain->getTwistTool(joint_pos_target,joint_vel_target).transpose();

  }
  catch(...)
  {
    ROS_ERROR("something wrong in target callback");
  }

}

InverseKinematicsQpPosVelEffController::~InverseKinematicsQpPosVelEffController()
{

}

void InverseKinematicsQpPosVelEffController::setConstraints(const Eigen::VectorXd& qmax, const Eigen::VectorXd& qmin,
                                                            const Eigen::VectorXd& Dqmax, const Eigen::VectorXd& DDqmax)
{
  m_ik_solver.setConstraints(qmax,qmin,Dqmax,DDqmax);
}

void InverseKinematicsQpPosVelEffController::setConstraints(const Eigen::Vector6d& constrained_twist)
{
  Eigen::MatrixXd  W  = speedLimit().asDiagonal() * (1. / speedLimit().maxCoeff() );
  Eigen::Matrix6Xd JJ = m_J * W * m_J.transpose();
  double           t  = constrained_twist.norm();
  if( t < 1e-4 )
  {
    CNR_ERROR_THROTTLE(*m_logger, 5.0, "The constrained veclocity is too low");
  }
  else
  {
    Eigen::Vector6d  u  =  constrained_twist / t;

    double  max_twist = double( u.transpose() * JJ * u ) * double( 1.0 * speedLimit().maxCoeff() );
    double  lambda = t > max_twist ? 1.0 : t / max_twist;

    m_ik_solver.setConstraints(upperLimit(), lowerLimit(), lambda * speedLimit(), accelerationLimit() );
  }
}

}

#undef GET_AND_RETURN
#undef GET_AND_DEFAULT
