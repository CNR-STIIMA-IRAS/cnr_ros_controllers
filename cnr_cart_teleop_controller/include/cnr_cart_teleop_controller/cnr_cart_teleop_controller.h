#ifndef CNR_CART_TELEOP_CONTROLLER__CNR_CART_TELEOP_CONTROLLER_H
#define CNR_CART_TELEOP_CONTROLLER__CNR_CART_TELEOP_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <subscription_notifier/subscription_notifier.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_core/primitives.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>

#include <inverse_kinematics_qp_controller/inverse_kinematics_qp_controller.h>

namespace cnr
{
namespace control
{

class CartTeleopController: public inverse_kinematics_qp::InverseKinematicsQpPosVelEffController
{

public:
    CartTeleopController();
    bool doInit      ( );
    bool doUpdate    (const ros::Time& time, const ros::Duration& period);
    bool doStarting  (const ros::Time& time);
    bool doStopping  (const ros::Time& time);

protected:

    std::shared_ptr<tf::TransformListener> m_listener;

    void setTargetCallback(const geometry_msgs::TwistStampedConstPtr& msg);

    Eigen::Matrix<double, 6, 1> m_target_twist;

    Eigen::Vector6d m_last_twist_in_b;
    Eigen::Vector6d m_twist_in_b;

    Eigen::Affine3d m_target_p;
    Eigen::Affine3d m_last_target_p;

    Eigen::Vector6d m_target_v;
    Eigen::Vector6d m_last_target_v;

    Eigen::VectorXd m_target_q;
    Eigen::VectorXd m_last_target_q;
};

}  // namespace control
}  // namespace cnr







#endif  // CNR_CART_TELEOP_CONTROLLER__CNR_CART_TELEOP_CONTROLLER_H
