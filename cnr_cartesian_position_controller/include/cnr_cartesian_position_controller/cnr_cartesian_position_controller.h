#ifndef cnr_cartesian_velocity_controller__20188101642
#define cnr_cartesian_velocity_controller__20188101642

#include <cmath>
#include <Eigen/Core>

#include <tf/transform_listener.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>

#include <actionlib/server/action_server.h>
#include <cnr_cartesian_position_controller/RelativeMoveAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <rosdyn_core/frame_distance.h>
namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The CartesianPositionController class
 */
class CartesianPositionController:
    public cnr::control::JointCommandController<
                  hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  CartesianPositionController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  tf::TransformListener listener_;

  std::shared_ptr<actionlib::ActionServer<cnr_cartesian_position_controller::RelativeMoveAction>>             m_as;
  std::shared_ptr<actionlib::ActionServer<cnr_cartesian_position_controller::RelativeMoveAction>::GoalHandle> m_gh;

  std::mutex mtx_;
  Eigen::Vector6d last_twist_of_in_b_;

  double max_cart_lin_vel_;
  double max_cart_lin_acc_;
  double max_cart_ang_vel_;
  double max_cart_ang_acc_;
  double m_clik_gain;
  double m_lin_tolerance=0.001;
  double m_ang_tolerance=0.01;
  bool m_stop_thread;
  std::thread m_as_thread;
  Eigen::Affine3d T_base_target_;
  Eigen::Affine3d T_base_t_;
  double target_velocity_=0.001;

  void actionGoalCallback   (actionlib::ActionServer<cnr_cartesian_position_controller::RelativeMoveAction>::GoalHandle gh);
  void actionCancelCallback (actionlib::ActionServer<cnr_cartesian_position_controller::RelativeMoveAction>::GoalHandle gh);
  void actionThreadFunction ( );

};


}
}



#endif
