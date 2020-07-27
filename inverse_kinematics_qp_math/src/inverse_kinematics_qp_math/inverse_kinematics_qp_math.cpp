#include <inverse_kinematics_qp_math/inverse_kinematics_qp_math.h>

namespace inverse_kinematics_qp
{
namespace math
{

InverseKinematicsQP::InverseKinematicsQP()
{
  m_is_invariance_active=true;
  ROS_INFO("Inverse kinematic solver created.");
}

void InverseKinematicsQP::updateMatrices()
{
  m_sol.resize(m_nax);
  m_sol.setZero();

  Eigen::MatrixXd I_df(m_nax,m_nax);
  I_df.setIdentity();

  m_CE.resize(m_nax,m_select_task_axis_matrix.rows());
  m_CE.setZero();
  m_ce0.resize(m_select_task_axis_matrix.rows());
  m_ce0.setZero();

  if (!m_is_invariance_active)
  {
    m_CI.resize(m_nax,6*m_nax);
    m_CI << m_dt*I_df, -m_dt*I_df, I_df, -I_df, I_df, -I_df; // pos, vel, acc limits
    m_ci0.resize(6*m_nax);
    m_ci0.setZero();
  }
  else
  {
    m_CI.resize(m_nax,8*m_nax);
    Eigen::MatrixXd marco=(0.99*m_dt*(m_DDqmax.cwiseQuotient(m_Dqmax))).asDiagonal();
    m_CI << m_dt*I_df, -m_dt*I_df, I_df, -I_df, I_df, -I_df, I_df+marco.matrix(), -I_df-marco; // pos, vel, acc, invariance limits
    m_ci0.resize(8*m_nax);
    m_ci0.setZero();
  }

  m_H_fixed.resize(m_nax,m_nax);
  m_f.resize(m_nax);
  m_f.setZero();
  if (!m_secondary_task.compare("minimum_velocity_norm"))
  {
    ROS_WARN_ONCE("Secondary objective for redundancy: minimum_velocity_norm");
    m_H_fixed.setIdentity();
  }
  else if (!m_secondary_task.compare("clearance"))
  {
    ROS_WARN_ONCE("Secondary objective for redundancy: clearance");
    m_H_fixed=(m_lambda_effort+m_lambda_return*m_dt)*I_df;
  }
  else if (!m_secondary_task.compare("custom"))
  {
    ROS_WARN_ONCE("Secondary objective for redundancy: custom task");
    m_H_fixed=m_lambda_effort*I_df;
  }
  else
  {
    ROS_WARN_ONCE("Undefined secondary objective for redundancy: using joint range availability as default");
    m_middle_configuration=0.5*(m_qmax+m_qmin);
    m_H_fixed=10.0*m_lambda_return*m_dt*I_df+m_lambda_effort*I_df;
    //m_f=10.0*m_lambda_return*(m_joint_position-m_middle_configuration);
  }
}

void InverseKinematicsQP::computeActualMatrices( const Eigen::VectorXd& targetDx, const Eigen::Affine3d& next_targetX, const Eigen::VectorXd& joint_position, const Eigen::VectorXd& joint_velocity)
{
  Eigen::Affine3d Tba=m_chain_task1->getTransformation(m_joint_position); // current ee pose
  Eigen::MatrixXd jacobian_ee = m_select_task_axis_matrix*(m_chain_task1->getJacobian(m_joint_position));
  Eigen::VectorXd task_error_in_b_full;
  rosdyn::getFrameDistance(next_targetX,Tba,task_error_in_b_full);
  Eigen::VectorXd task_error_in_b=m_select_task_axis_matrix*task_error_in_b_full;

  m_CE=jacobian_ee.transpose();
  m_ce0=-(m_select_task_axis_matrix*targetDx+m_lambda_clik*task_error_in_b);

  if (!m_is_invariance_active)
    m_ci0 << -m_qmin+m_joint_position, m_qmax-m_joint_position, m_Dqmax, m_Dqmax, -m_joint_velocity+m_dt*m_DDqmax, m_joint_velocity+m_dt*m_DDqmax; // pos, vel, acc limits
  else
    m_ci0 << -m_qmin+m_joint_position, m_qmax-m_joint_position, m_Dqmax, m_Dqmax, -m_joint_velocity+m_dt*m_DDqmax, m_joint_velocity+m_dt*m_DDqmax, 0.99*(m_DDqmax.cwiseQuotient(m_Dqmax)).asDiagonal()*(m_joint_position-m_qmin), -0.99*(m_DDqmax.cwiseQuotient(m_Dqmax)).asDiagonal()*(m_joint_position-m_qmax); // pos, vel, acc, invariance limits

  if (!m_secondary_task.compare("minimum_velocity_norm"))
  {
    m_H=m_H_fixed;
    m_f.setZero();
  }
  else if (!m_secondary_task.compare("clearance"))
  {
    Eigen::Affine3d Tbe=m_chain_task2->getTransformation(m_joint_position.head(m_nax_task2)); // current elbow pose
    Eigen::Vector3d Oelbow_in_b=Tbe.translation(); // current elbow position
    Eigen::MatrixXd jacobian_elbow(3,m_nax);
    jacobian_elbow << (m_chain_task2->getJacobian(m_joint_position.head(m_nax_task2))).block(0,0,3,m_nax_task2), Eigen::MatrixXd::Zero(3,m_nax-m_nax_task2);
    double distance= (Oelbow_in_b-m_obstacle_in_b).norm();
    double lambda_distance;
    if (distance<=m_clearance_threshold)
      lambda_distance=m_lambda_distance;
    else
      lambda_distance=0.0;
    m_H=m_H_fixed+lambda_distance*(-m_dt*(jacobian_elbow.transpose() * jacobian_elbow));
    m_f=lambda_distance*(-jacobian_elbow.transpose()*(Oelbow_in_b-m_obstacle_in_b) );
    m_f+=m_lambda_return*(m_joint_position-m_rest_configuration);
  }
  else if (!m_secondary_task.compare("custom"))
  {
    m_H=m_H_fixed+m_A_custom.transpose()*m_W_custom*m_A_custom;
    m_f=m_b_custom.transpose()*m_W_custom*m_A_custom;
  }
  else
  {
    m_H=m_H_fixed;
    m_f=10.0*m_lambda_return*(m_joint_position-m_middle_configuration);
  }

}

void InverseKinematicsQP::setConstraints ( const Eigen::VectorXd& qmax, const Eigen::VectorXd& qmin, const Eigen::VectorXd& Dqmax, const Eigen::VectorXd& DDqmax )
{
  m_qmax=qmax;
  m_qmin=qmin;
  m_Dqmax=Dqmax;
  m_DDqmax=DDqmax;
  m_are_matrices_updated=false;
}

void InverseKinematicsQP::enableInvariance(const bool enable_invariance)
{
  if (m_is_invariance_active!=enable_invariance)
  {
    m_is_invariance_active=enable_invariance;
    m_are_matrices_updated=false;
    ROS_INFO("Invariance flag updated. Execute update matrices to load the new options.");
  }
}

bool InverseKinematicsQP::isInvarianceEnabled()
{
  return m_is_invariance_active;
}

void InverseKinematicsQP::setWeigthFunction( const double& lambda_distance, const double& lambda_effort, const double& lambda_return, const double& lambda_clik )
{
  m_lambda_distance=lambda_distance;
  m_lambda_effort=lambda_effort;
  m_lambda_return=lambda_return;
  m_lambda_clik=lambda_clik;
  m_are_matrices_updated=false;
}

// FIX: not working with equality constraints
bool InverseKinematicsQP::computedUncostrainedSolution(  const Eigen::VectorXd& targetDx,
                                    const Eigen::Affine3d& next_targetX,
                                    const Eigen::VectorXd& current_q,
                                    const Eigen::VectorXd& current_Dq,
                                    Eigen::VectorXd& next_vel)
{
  computeActualMatrices(targetDx,next_targetX,current_q,current_Dq);

  m_svd.compute( m_H, Eigen::ComputeThinU | Eigen::ComputeThinV );
  m_sol=-m_svd.solve(m_f);
  next_vel=m_sol.head(m_nax);

  return true;
}

bool InverseKinematicsQP::computedCostrainedSolution(  const Eigen::VectorXd& targetDx,
                                    const Eigen::Affine3d& next_targetX,
                                    const Eigen::VectorXd& current_q,
                                    const Eigen::VectorXd& current_Dq,
                                    Eigen::VectorXd& next_vel)
{
  computeActualMatrices(targetDx,next_targetX,current_q,current_Dq);

  Eigen::solve_quadprog(m_H,m_f,m_CE,m_ce0,m_CI,m_ci0,m_sol );
  next_vel=m_sol;

  return true;
}

void InverseKinematicsQP::computeTaskSelectionMatrix(const std::vector<int>& selection_vector)
{
  std::vector<int> select_task_axis_vector=selection_vector;
  if (select_task_axis_vector.size()>6)
  {
    ROS_ERROR("InverseKinematicsQP: length of 'select_task_axis_vector' greater than 6. Exceeding elements will be neglected.");
    select_task_axis_vector.resize(6);
  }
  else if (select_task_axis_vector.size()<6)
  {
    ROS_ERROR("InverseKinematicsQP: length of 'select_task_axis_vector' smaller than 6. Missing elements will be set to zero.");
    unsigned int size_vec=select_task_axis_vector.size();
    select_task_axis_vector.resize(6);
    for (unsigned int idx=0;idx<6-size_vec;idx++)
      select_task_axis_vector.at(size_vec+idx)=0;
  }
  std::vector<int> indices;
  for (unsigned int idx=0;idx<select_task_axis_vector.size();idx++)
    if (select_task_axis_vector.at(idx)==1)
      indices.push_back(idx);
  m_select_task_axis_matrix.resize(indices.size(),select_task_axis_vector.size());
  for (unsigned int idx=0;idx<indices.size();idx++)
    m_select_task_axis_matrix.row(idx)=(Eigen::MatrixXd::Identity(6,6)).row(indices.at(idx));
}

void InverseKinematicsQP::setObstaclePosition(const Eigen::Affine3d& obstacle_pose_in_b)
{
  m_obstacle_in_b=obstacle_pose_in_b.translation();
}

void InverseKinematicsQP::setRestConfiguration(const Eigen::VectorXd& rest_configuration)
{
  assert(rest_configuration.size()==m_nax);
  m_rest_configuration=rest_configuration;
}

void InverseKinematicsQP::setSecondaryTask(const std::string& secondary_task)
{
  m_secondary_task=secondary_task;
}

void InverseKinematicsQP::setClearanceThreshold( const double& clearance_threshold )
{
  m_clearance_threshold=clearance_threshold;
}

void InverseKinematicsQP::setCustomSecondaryTaskMatrices(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::MatrixXd& W)
{
  m_A_custom=A;
  m_b_custom=b;
  m_W_custom=W;
}

void InverseKinematicsQP::setInitialState( const Eigen::VectorXd& joint_position, const Eigen::VectorXd& joint_velocity )
{
  assert(joint_position.size()==m_nax);
  assert(joint_velocity.size()==m_nax);
  m_joint_position=joint_position;
  m_joint_velocity=joint_velocity;

}

void InverseKinematicsQP::updateState( const Eigen::VectorXd& next_vel )
{
  m_joint_position+=next_vel*m_dt;
  m_joint_velocity=next_vel;
}

void InverseKinematicsQP::setSamplingPeriod( const double& computing_period )
{
  m_dt=computing_period;
}

Eigen::VectorXd InverseKinematicsQP::getJointPosition()
{
  return m_joint_position;
}

Eigen::VectorXd InverseKinematicsQP::getJointVelocity()
{
  return m_joint_velocity;
}

Eigen::Affine3d InverseKinematicsQP::getPoseTask1()
{
  return m_chain_task1->getTransformation(m_joint_position);
}

Eigen::Vector6d InverseKinematicsQP::getTwistTask1()
{
  return m_chain_task1->getTwistTool(m_joint_position,m_joint_velocity);
}

void InverseKinematicsQP::setDynamicsChainTask1(const boost::shared_ptr<rosdyn::Chain>&  chain)
{
  m_chain_task1=chain;
}

boost::shared_ptr<rosdyn::Chain> InverseKinematicsQP::getDynamicsChainTask1()
{
 return m_chain_task1;
}

void InverseKinematicsQP::setDynamicsChainTask2(const boost::shared_ptr<rosdyn::Chain>&  chain)
{
  m_chain_task2=chain;
}

boost::shared_ptr<rosdyn::Chain> InverseKinematicsQP::getDynamicsChainTask2()
{
 return m_chain_task2;
}

void InverseKinematicsQP::setAxisNumberTask1( const unsigned int& nax )
{
  m_nax=nax;
}

void InverseKinematicsQP::setAxisNumberTask2( const unsigned int& nax )
{
  m_nax_task2=nax;
}

void InverseKinematicsQP::setClearanceOptions( const boost::shared_ptr<rosdyn::Chain>&  chain, const unsigned int& nax, const double& clearance_threshold, const Eigen::VectorXd& rest_configuration, const Eigen::Affine3d& obstacle_pose_in_b )
{
  if (m_secondary_task.compare("clearance"))
    ROS_WARN("You are setting clearance options but the secondary task is not set to 'clearance'.");
  setDynamicsChainTask2(chain);
  setAxisNumberTask2(nax);
  setClearanceThreshold(clearance_threshold);
  setRestConfiguration(rest_configuration);
  setObstaclePosition(obstacle_pose_in_b);
}

double InverseKinematicsQP::computeClosestObstacle(const std::vector<Eigen::Affine3d>& obstacle_pose_array, Eigen::Affine3d& closest_obstacle)
{
  Eigen::VectorXd robotPoint(3);
  robotPoint=(m_chain_task2->getTransformation(m_joint_position.head(m_nax_task2))).translation();
  double min_distance=std::numeric_limits<double>::infinity();
  for (unsigned int idx=0;idx<obstacle_pose_array.size();idx++)
  {
    double distance=(obstacle_pose_array.at(idx).translation()-robotPoint).norm();
    if (distance<min_distance)
    {

      closest_obstacle=obstacle_pose_array.at(idx);
      min_distance=distance;

    }
  }

  return min_distance;
}

void InverseKinematicsQP::printALL()
{
  ROS_INFO("m_nax=%d", m_nax );
  ROS_INFO("m_nax_task2=%d", m_nax_task2 );
  ROS_INFO("m_dt=%f", m_dt );

  ROS_INFO_STREAM("m_H_fixed=\n" << m_H_fixed);
  ROS_INFO_STREAM("m_H_variable=\n" << m_H_variable);
  ROS_INFO_STREAM("m_H=\n" << m_H);
  ROS_INFO_STREAM("m_f=" << m_f.transpose());

  ROS_INFO_STREAM("m_CE=\n" << m_CE);
  ROS_INFO_STREAM("m_ce0=" << m_ce0.transpose());
  ROS_INFO_STREAM("m_CI=\n" << m_CI);
  ROS_INFO_STREAM("m_ci0=" << m_ci0.transpose());

  ROS_INFO_STREAM("m_qmax=" << m_qmax.transpose());
  ROS_INFO_STREAM("m_qmin=" << m_qmin.transpose());
  ROS_INFO_STREAM("m_Dqmax=" << m_Dqmax.transpose());
  ROS_INFO_STREAM("m_DDqmax=" << m_DDqmax.transpose());

  ROS_INFO_STREAM("m_svd_U=" << m_svd.matrixU());
  ROS_INFO_STREAM("m_svd_V=" << m_svd.matrixV());

  ROS_INFO_STREAM("m_sol=" << m_sol.transpose());

  ROS_INFO("m_lambda_distance=%f", m_lambda_distance );
  ROS_INFO("m_lambda_effort=%f", m_lambda_effort );
  ROS_INFO("m_lambda_return=%f", m_lambda_return );
  ROS_INFO("m_lambda_clik=%f", m_lambda_clik );
  ROS_INFO("m_clearance_threshold=%f", m_clearance_threshold );

  ROS_INFO_STREAM("m_select_task_axis_matrix=\n" << m_select_task_axis_matrix);
  ROS_INFO_STREAM("m_secondary_task=" << m_secondary_task.c_str());

  ROS_INFO_STREAM("m_A_custom=\n" << m_A_custom);
  ROS_INFO_STREAM("m_b_custom=" << m_b_custom.transpose());
  ROS_INFO_STREAM("m_W_custom=\n" << m_W_custom);

  ROS_INFO_STREAM("m_joint_position=" << m_joint_position.transpose());
  ROS_INFO_STREAM("m_joint_velocity=" << m_joint_velocity.transpose());
  ROS_INFO_STREAM("m_rest_configuration=" << m_rest_configuration.transpose());
  ROS_INFO_STREAM("m_middle_configuration=" << m_middle_configuration.transpose());
  ROS_INFO_STREAM("m_obstacle_in_b=" << m_obstacle_in_b.transpose());

}

}
}
