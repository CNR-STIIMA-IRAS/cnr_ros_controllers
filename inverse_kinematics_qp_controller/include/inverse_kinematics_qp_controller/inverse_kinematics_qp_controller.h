#ifndef INVERSE_KINEMATICS_QP_CONTROLLER____
#define INVERSE_KINEMATICS_QP_CONTROLLER____

#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <inverse_kinematics_qp_math/inverse_kinematics_qp_math.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <subscription_notifier/subscription_notifier.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <eigen_conversions/eigen_msg.h>
#include <name_sorting/name_sorting.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace inverse_kinematics_qp
{

  class InverseKinematicsQpPosVelEffController : public cnr_controller_interface::JointCommandController<hardware_interface::PosVelEffJointInterface>
  {
  public:
    virtual bool doInit();
    virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
    virtual bool doStarting(const ros::Time& time);
    virtual bool doStopping(const ros::Time& time);

    void    setConstraints(const Eigen::VectorXd& qmax, const Eigen::VectorXd& qmin,
                           const Eigen::VectorXd& Dqmax, const Eigen::VectorXd& DDqmax);

    void    setConstraints(const Eigen::Vector6d& constrained_twist_max);


  protected:

    void obstacleCallback(const geometry_msgs::PoseArrayConstPtr& msg);
    // void customMatricesCallback(const Eigen::MatrixXd msg); // dovrei creare un msg con dentro una matrice e un vettore
    void targetPoseCallback(const geometry_msgs::PoseConstPtr& msg);
    void targetTwistCallback(const geometry_msgs::TwistConstPtr& msg);
    void targetJointCallback(const sensor_msgs::JointStateConstPtr& msg);

    ~InverseKinematicsQpPosVelEffController();


    //std::vector<std::string> m_joint_names;
    std::vector<std::string> m_joints_names_task2;
    size_t m_nAx2;

    Eigen::Vector6d m_target_Dx;
    Eigen::Affine3d m_next_X;
    Eigen::VectorXd m_next_vel;

    Eigen::Affine3d m_obstacle_pose_in_b;
    std::vector<Eigen::Affine3d> m_obstacle_pose_vector; // allocator???

    inverse_kinematics_qp::math::InverseKinematicsQP m_ik_solver;

    double m_max_cart_lin_vel;
    double m_max_cart_lin_acc;
    double m_max_cart_ang_vel;
    double m_max_cart_ang_acc;


  };

};



#endif
