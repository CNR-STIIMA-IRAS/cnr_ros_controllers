#ifndef cnr_cartesian_velocity_controller__20188101642
#define cnr_cartesian_velocity_controller__20188101642

#include <cmath>
#include <Eigen/Core>

#include <tf/transform_listener.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The CartesianVelocityController class
 */
class CartesianVelocityController:
    public cnr::control::JointCommandController<
                  hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  CartesianVelocityController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:
  void twistSetPointCallback(const geometry_msgs::TwistStampedConstPtr& msg);

  tf::TransformListener listener_;

  size_t m_singularity_pub;
  int singularity_times = 0;

  std::mutex mtx_;
  Eigen::Vector6d twist_of_t_in_b_;
  Eigen::Vector6d last_twist_of_in_b_;

  double max_cart_lin_vel_;
  double max_cart_lin_acc_;
  double max_cart_ang_vel_;
  double max_cart_ang_acc_;
};


}
}



#endif
