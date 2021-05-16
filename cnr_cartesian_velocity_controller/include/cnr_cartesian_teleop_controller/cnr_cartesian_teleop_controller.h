#ifndef cnr_cartesian_teleop_controller__20188101642
#define cnr_cartesian_teleop_controller__20188101642

#include <cmath>
#include <Eigen/Core>

#include <ros/time.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>

#include <state_space_filters/filtered_values.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The CartesianTeleopController class
 */
class CartesianTeleopController:
    public cnr::control::JointCommandController<
                  hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  CartesianTeleopController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);
  void callback(const geometry_msgs::TwistStampedConstPtr &msg);

protected:

  std::mutex m_mtx;

  rosdyn::VectorXd m_vel_sp;
  rosdyn::VectorXd m_pos_sp;
  rosdyn::VectorXd m_vel_sp_last;

  double m_max_cart_lin_vel;
  double m_max_cart_lin_acc;
  double m_max_cart_ang_vel;
  double m_max_cart_ang_acc;

  std::shared_ptr<tf::TransformListener> m_listener;
  Eigen::Vector6d m_twist_of_t_in_b;
  Eigen::Vector6d m_last_twist_of_in_b;

};


}
}



#endif
