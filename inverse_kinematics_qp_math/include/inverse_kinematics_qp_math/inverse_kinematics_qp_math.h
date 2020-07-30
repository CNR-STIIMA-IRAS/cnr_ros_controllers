#ifndef __inverse_kinematics_qp_math__
#define __inverse_kinematics_qp_math__

#include "Eigen/Dense"
#include <ros/console.h>
#include <eigen_matrix_utils/eiquadprog.hpp>
#include <rosdyn_core/primitives.h>
#include <rosdyn_core/frame_distance.h>


namespace inverse_kinematics_qp
{
namespace math
{

class InverseKinematicsQP
{
protected:
  bool m_are_matrices_updated;

  rosdyn::ChainPtr m_chain_task1;
  rosdyn::ChainPtr m_chain_task2;

  unsigned int m_nax; //nmber of joints
  unsigned int m_nax_task2; //number of joints from base to elbow
  double m_dt;

  bool m_is_invariance_active;

  Eigen::MatrixXd m_H_fixed;
  Eigen::MatrixXd m_H_variable;

  Eigen::MatrixXd m_H;
  Eigen::VectorXd m_f;

  Eigen::MatrixXd m_CE;
  Eigen::VectorXd m_ce0;
  Eigen::MatrixXd m_CI;
  Eigen::VectorXd m_ci0;

  Eigen::VectorXd m_qmax;
  Eigen::VectorXd m_qmin;
  Eigen::VectorXd m_Dqmax;
  Eigen::VectorXd m_DDqmax;
  // Eigen::VectorXd m_tau_max;

  Eigen::JacobiSVD<Eigen::MatrixXd>  m_svd;
  Eigen::VectorXd m_sol;

  double m_lambda_distance;
  double m_lambda_effort;
  double m_lambda_return;
  double m_lambda_clik;
  double m_clearance_threshold;

  Eigen::MatrixXd m_select_task_axis_matrix;
  std::string m_secondary_task;

  Eigen::MatrixXd m_A_custom;
  Eigen::VectorXd m_b_custom;
  Eigen::MatrixXd m_W_custom;

  Eigen::VectorXd m_joint_position;
  Eigen::VectorXd m_joint_velocity;

  Eigen::VectorXd m_rest_configuration;
  Eigen::VectorXd m_middle_configuration;
  Eigen::VectorXd m_obstacle_in_b;

  void computeActualMatrices( const Eigen::VectorXd& targetDx,
                              const Eigen::Affine3d& next_targetX,
                              const Eigen::VectorXd& joint_position,
                              const Eigen::VectorXd& joint_velocity);

public:
  InverseKinematicsQP();

  void updateMatrices();

  void setConstraints( const Eigen::VectorXd& qmax,
                       const Eigen::VectorXd& qmin,
                       const Eigen::VectorXd& Dqmax,
                       const Eigen::VectorXd& DDqmax);

  void enableInvariance(const bool enable_invariance);

  bool isInvarianceEnabled();

  void setWeigthFunction( const double& lambda_distance,
                          const double& lambda_effort,
                          const double& lambda_return,
                          const double& lambda_clik );

  bool computedUncostrainedSolution(  const Eigen::VectorXd& targetDx,
                                      const Eigen::Affine3d& next_targetX,
                                      const Eigen::VectorXd& current_q,
                                      const Eigen::VectorXd& current_Dq,
                                      Eigen::VectorXd& next_vel );
  bool computedCostrainedSolution(  const Eigen::VectorXd& targetDx,
                                      const Eigen::Affine3d& next_targetX,
                                      const Eigen::VectorXd& current_q,
                                      const Eigen::VectorXd& current_Dq,
                                      Eigen::VectorXd& next_vel );

  //void setInitialState(const Eigen::VectorXd& x0);
  void setObstaclePosition(const Eigen::Affine3d& obstacle_pose_in_b);
  void setRestConfiguration(const Eigen::VectorXd& rest_configuration);
  void setSecondaryTask(const std::string& secondary_task);
  void setCustomSecondaryTaskMatrices(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::MatrixXd& W);
  void computeTaskSelectionMatrix(const std::vector<int>& selection_vector);
  void setInitialState( const Eigen::VectorXd& joint_position, const Eigen::VectorXd& joint_velocity );
  void updateState( const Eigen::VectorXd& next_vel );
  void setSamplingPeriod( const double& computing_period );
  void setClearanceThreshold( const double& clearance_threshold );

  double computeClosestObstacle(const std::vector<Eigen::Affine3d>& obstacle_pose_array, Eigen::Affine3d& closest_obstacle);

  Eigen::VectorXd getJointPosition();
  Eigen::VectorXd getJointVelocity();
  Eigen::Affine3d getPoseTask1();
  Eigen::Vector6d getTwistTask1();

  void setDynamicsChainTask1(const rosdyn::ChainPtr&  chain);
  void setDynamicsChainTask2(const rosdyn::ChainPtr&  chain);
  void setAxisNumberTask1( const unsigned int& nax );
  void setAxisNumberTask2( const unsigned int& nax );

  rosdyn::ChainPtr getDynamicsChainTask1();
  rosdyn::ChainPtr getDynamicsChainTask2();

  void setClearanceOptions( const rosdyn::ChainPtr&  chain,
                            const unsigned int& nax,
                            const double& clearance_threshold,
                            const Eigen::VectorXd& rest_configuration,
                            const Eigen::Affine3d& obstacle_pose_in_b );

  void printALL();

};

}
}

#endif
