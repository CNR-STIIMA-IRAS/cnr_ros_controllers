#include <cnr_logger/cnr_logger.h>
#include <inverse_kinematics_qp_controller/inverse_kinematics_qp_controller.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(inverse_kinematics_qp::InverseKinematicsQpPosVelEffController, controller_interface::ControllerBase);

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
  CNR_TRACE_START(*m_logger, "Initializing Inverse Kinematics QP controller '" + m_controller_nh.getNamespace() + "'");

  bool cartesian_reference;
  if (!m_controller_nh.getParam("cartesian_reference",cartesian_reference))
  {
    ROS_WARN("cartesian_reference not defined, using joint reference");
    cartesian_reference=false;
  }
  if (cartesian_reference)
  {
    add_subscriber( "obstacle_subscriber",
          "/obstacle_in_b",1,&inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::obstacleCallback,this);
    add_subscriber("target_pose_subscriber",
        "target_pose",1,&inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::targetPoseCallback,this);
    add_subscriber("target_twist_subscriber",
        "target_twist",1,&inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::targetTwistCallback,this);
  }
  else
  {
    std::string joint_target;
    if (!m_controller_nh.getParam("joint_target_topic", joint_target))
    {
      ROS_WARN_STREAM(m_controller_nh.getNamespace()+"/'joint_target' does not exist. Default value 'joint_target' superimposed.");
      joint_target = "joint_target";
    }
    add_subscriber("joint_reference_subscriber",
      joint_target,1,&inverse_kinematics_qp::InverseKinematicsQpPosVelEffController::targetJointCallback,this);
  }

  if (!m_controller_nh.getParam("controlled_joint",m_joint_names))
  {
    ROS_FATAL("controlled_joint not defined");
    return false;
  }
  std::string base_frame;
  std::string tool_frame;
  if (!m_controller_nh.getParam("base_frame", base_frame) || !m_controller_nh.getParam("tool_frame", tool_frame))
  {
    ROS_FATAL("base_frame or tool_frame not defined");
    return false;
  }

  m_base_frame=base_frame;


  m_model.initParam("robot_description");
  m_nAx=m_joint_names.size();
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  boost::shared_ptr<rosdyn::Chain> chainTask1 = rosdyn::createChain(m_model,base_frame,tool_frame,grav);
  chainTask1->setInputJointsName(m_joint_names);

  std::vector<double> rest_config_std(m_nAx);
  Eigen::VectorXd rest_config(m_nAx);
  if (!m_controller_nh.getParam("rest_config", rest_config_std))
  {
    ROS_ERROR("unable to load param: rest_config. Loading deafult value: 0");
    for (unsigned int idx=0;idx<rest_config_std.size();idx++)
      rest_config_std.at(idx)=0.0;
  }
  for (unsigned int idx=0;idx<rest_config_std.size();idx++)
    rest_config(idx)=rest_config_std.at(idx);

  Eigen::VectorXd qmax(m_nAx);
  Eigen::VectorXd qmin(m_nAx);
  Eigen::VectorXd Dqmax(m_nAx);
  Eigen::VectorXd DDqmax(m_nAx);
  qmax.setConstant(3);
  qmin.setConstant(-3);
  Dqmax.setConstant(0.5);
  DDqmax.setConstant(4);

  m_jh.resize(m_nAx);
  for (unsigned int iAx=0; iAx<m_nAx; iAx++)
  {
    m_jh.at(iAx)=m_hw->getHandle(m_joint_names.at(iAx));
    qmax(iAx) = m_model.getJoint(m_joint_names.at(iAx))->limits->upper;
    qmin(iAx) = m_model.getJoint(m_joint_names.at(iAx))->limits->lower;

    if ((qmax(iAx)==0) && (qmin(iAx)==0))
    {
      qmin(iAx)=1.0e9;
      qmin(iAx)=-1.0e9;
      ROS_INFO("upper and lower limits are both equal to 0, set +/- 1.0e9");
    }

    bool has_velocity_limits;
    if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/has_velocity_limits",has_velocity_limits))
      has_velocity_limits=false;
    bool has_acceleration_limits;
    if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/has_acceleration_limits",has_acceleration_limits))
      has_acceleration_limits=false;

    Dqmax(iAx)= m_model.getJoint(m_joint_names.at(iAx))->limits->velocity;
    if (has_velocity_limits)
    {
      double vel;
      if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_velocity",vel))
      {
        ROS_ERROR_STREAM("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_velocity is not defined");
        return false;
      }
      if (vel<Dqmax(iAx))
        Dqmax(iAx)=vel;
    }

    if (has_acceleration_limits)
    {
      double acc;
      if (!m_root_nh.getParam("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_acceleration",acc))
      {
        ROS_ERROR_STREAM("/robot_description_planning/joint_limits/"+m_joint_names.at(iAx)+"/max_acceleration is not defined");
        return false;
      }
      DDqmax(iAx)=acc;
    }
    else
      DDqmax(iAx)=10*Dqmax(iAx);
  }



  double dt; // [ms]
  double lambda_effort;
  double lambda_clik;
  double lambda_distance;
  double lambda_return;
  if (!m_root_nh.getParam("sampling_period",dt))
  {
    dt=0.008;
    ROS_WARN("unable to load param: dt. Loading deafult value %f",dt);
  }
  if (!m_controller_nh.getParam("lambda_effort",lambda_effort))
  {
    lambda_effort=0.001;
    ROS_WARN("unable to load param: lambda_effort. Loading deafult value %f",lambda_effort);
  }
  if (!m_controller_nh.getParam("lambda_clik",lambda_clik))
  {
    lambda_clik=0.1;
    ROS_WARN("unable to load param: lambda_clik. Loading deafult value %f",lambda_clik);
  }
  if (!m_controller_nh.getParam("lambda_distance",lambda_distance))
  {
    lambda_distance=0.1;
    ROS_WARN("unable to load param: lambda_distance. Loading deafult value %f",lambda_distance);
  }
  if (!m_controller_nh.getParam("lambda_return",lambda_return))
  {
    lambda_return=0.008;
    ROS_WARN("unable to load param: lambda_return. Loading deafult value %f",lambda_return);
  }

  std::vector<int> select_task_axis;
  if (!m_controller_nh.getParam("select_task_axis",select_task_axis))
  {
    ROS_WARN("unable to load param: select_task_axis. Default: 1,1,1,1,1,1");
    select_task_axis={1,1,1,1,1,1};
  }
  if (select_task_axis.size()!=6)
  {
    ROS_WARN("length of param 'select_task_axis' expected to be equal to 6 but is equal to %zu",select_task_axis.size());
    return false;
  }

  std::string secondary_task;
  if (!m_controller_nh.getParam("secondary_task",secondary_task))
  {
    secondary_task="other";
    ROS_WARN("unable to load param: secondary_task. Default: %s",secondary_task.c_str());
  }

  if (!secondary_task.compare("clearance"))
  {
    std::string task2_frame;

    if (!m_controller_nh.getParam("elbow_frame", task2_frame))
    {
      ROS_FATAL("task2_frame or nAx2 not defined");
      return false;
    }

    boost::shared_ptr<rosdyn::Chain> chainTask2 = rosdyn::createChain(m_model,base_frame,task2_frame,grav);

    m_nAx2=chainTask2->getActiveJointsNumber();
    for (unsigned int idx=0;idx<m_nAx2;idx++)
    {
      m_joints_names_task2.push_back(m_joint_names.at(idx));
    }


    double clearance_threshold;
    if (!m_controller_nh.getParam("clearance_threshold",clearance_threshold))
    {
      clearance_threshold=1;
      ROS_ERROR("unable to load param: clearance_threshold. Loading deafult value %f",clearance_threshold);
    }

    Eigen::VectorXd obstacle_in_b(3);
    obstacle_in_b(0)=1.5;
    obstacle_in_b(1)=-0.5;
    obstacle_in_b(2)=-0.5;
    m_obstacle_pose_in_b.linear().setIdentity();
    m_obstacle_pose_in_b.translation()=obstacle_in_b;
    m_obstacle_pose_vector.push_back(m_obstacle_pose_in_b);
    m_ik_solver.setClearanceOptions(chainTask2,m_nAx2,clearance_threshold,rest_config,m_obstacle_pose_in_b);

  }

  m_ik_solver.setAxisNumberTask1(m_nAx);
  m_ik_solver.setDynamicsChainTask1(chainTask1);
  m_ik_solver.setConstraints(qmax,qmin,Dqmax,DDqmax);
  m_ik_solver.setSamplingPeriod(dt);
  m_ik_solver.computeTaskSelectionMatrix(select_task_axis);
  m_ik_solver.setSecondaryTask(secondary_task);
  m_ik_solver.setWeigthFunction(lambda_distance,lambda_effort,lambda_return,lambda_clik);
  m_ik_solver.updateMatrices();

  CNR_RETURN_TRUE(*m_logger);
}

bool InverseKinematicsQpPosVelEffController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger, "Starting Inverse Kinematics QP controller '" + m_controller_nh.getNamespace() + "'");

  Eigen::VectorXd qini(m_nAx);
  Eigen::VectorXd Dqini(m_nAx);
  for (unsigned int idx=0;idx<m_nAx;idx++)
  {
    qini(idx)=m_hw->getHandle(m_joint_names.at(idx)).getPosition();
    Dqini(idx)=m_hw->getHandle(m_joint_names.at(idx)).getVelocity();
  }

  m_ik_solver.setInitialState(qini,Dqini);
  m_next_vel.resize(m_nAx);
  m_next_vel.setZero();
  m_next_X=m_ik_solver.getPoseTask1();
  m_target_Dx.setZero();

  // seto obstacle e/o custom matrices

  CNR_RETURN_TRUE(*m_logger);
}

bool InverseKinematicsQpPosVelEffController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger, "Stopping Inverse Kinematics QP controller '" + m_controller_nh.getNamespace()+"'");
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

    m_ik_solver.computedCostrainedSolution(m_target_Dx,m_next_X,m_ik_solver.getJointPosition(),m_ik_solver.getJointVelocity(),m_next_vel);
    m_ik_solver.updateState(m_next_vel);


    for (unsigned int iDim = 0;iDim<m_nAx;iDim++)
    {
      m_hw->getHandle(m_joint_names.at(iDim)).setCommandPosition(m_ik_solver.getJointPosition()(iDim));
      m_hw->getHandle(m_joint_names.at(iDim)).setCommandVelocity(m_ik_solver.getJointVelocity()(iDim));
      m_hw->getHandle(m_joint_names.at(iDim)).setCommandEffort(0.0);
    }
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
    if (m_base_frame.compare(msg->header.frame_id))
      ROS_ERROR("Obstacle frame expected to be '%s' but received msg frame is '%s'. Message neglected.",m_base_frame.c_str(),(msg->header.frame_id).c_str());
    else
    {
      m_obstacle_pose_vector.clear();
      for (unsigned int idx=0;idx<msg->poses.size();idx++)
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
    if (!name_sorting::permutationName(m_joint_names,tmp_msg.name,tmp_msg.position,tmp_msg.velocity))
    {
      ROS_ERROR("joints not found");
      return;
    }

    boost::shared_ptr<rosdyn::Chain> chain=m_ik_solver.getDynamicsChainTask1();
    Eigen::VectorXd joint_pos_target(m_nAx);
    Eigen::VectorXd joint_vel_target(m_nAx);
    for (unsigned int idx=0;idx<m_nAx;idx++)
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

}
