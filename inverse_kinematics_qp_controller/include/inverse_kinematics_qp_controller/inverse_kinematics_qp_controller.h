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
#include <cnr_controller_interface/cnr_joint_controller_interface.h>

namespace inverse_kinematics_qp
{

  class InverseKinematicsQpPosVelEffController : public cnr_controller_interface::JointController<hardware_interface::PosVelEffJointInterface>
  {
  public:
    bool doInit();
    bool doUpdate(const ros::Time& time, const ros::Duration& period);
    bool doStarting(const ros::Time& time);
    bool doStopping(const ros::Time& time);

  protected:

    void obstacleCallback(const geometry_msgs::PoseArrayConstPtr& msg);
    // void customMatricesCallback(const Eigen::MatrixXd msg); // dovrei creare un msg con dentro una matrice e un vettore
    void targetPoseCallback(const geometry_msgs::PoseConstPtr& msg);
    void targetTwistCallback(const geometry_msgs::TwistConstPtr& msg);
    void targetJointCallback(const sensor_msgs::JointStateConstPtr& msg);


    std::vector<hardware_interface::PosVelEffJointHandle> m_jh;
    urdf::Model m_model;
    ~InverseKinematicsQpPosVelEffController();

    ros::NodeHandle m_root_nh;
    ros::NodeHandle m_controller_nh;

    //std::vector<std::string> m_joint_names;
    std::vector<std::string> m_joints_names_task2;
    std::string m_base_frame;
    unsigned int m_nAx;
    unsigned int m_nAx2;

    Eigen::Vector6d m_target_Dx;
    Eigen::Affine3d m_next_X;
    Eigen::VectorXd m_next_vel;

    Eigen::Affine3d m_obstacle_pose_in_b;
    std::vector<Eigen::Affine3d> m_obstacle_pose_vector; // allocator???

    inverse_kinematics_qp::math::InverseKinematicsQP m_ik_solver;


  };

};



#endif
