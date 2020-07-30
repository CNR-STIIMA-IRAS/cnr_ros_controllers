#ifndef cart_motion_ctrl_20190325
#define cart_motion_ctrl_20190325

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <rosdyn_core/primitives.h>
#include <ros/callback_queue.h>
#include <actionlib/server/action_server.h>
#include <thread>
#include <tf/transform_listener.h>
#include <cartesian_motion_msgs/cartMotionAction.h>
#include <cartesian_motion_msgs/simpleTouchAction.h>
#include <cnr_cart_teleop_controller/cnr_cart_teleop_controller.h>

#if ROS_VERSION_MINIMUM(1, 14, 1)
# include <memory>
#else
#endif


namespace cnr
{
namespace control
{
class CartMotionController: public cnr::control::CartTeleopController
{

public:

  CartMotionController( ) = default;
  bool doInit     ( );
  bool doUpdate   ( const ros::Time& time, const ros::Duration& period );
  bool doStarting ( const ros::Time& time );
  bool doStopping ( const ros::Time& time );

protected:

  ros::Subscriber wrench_sub_;

  std::shared_ptr<tf::TransformListener> listener_;

  std::shared_ptr<actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>>             as_;
  std::shared_ptr<actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>::GoalHandle> gh_;
  std::thread               as_thread_;

  double                            goal_linear_tollerance_;
  double                            goal_angular_tollerance_;
  Eigen::Matrix<double,7,1>         goal_displacement_;

  bool                              goal_reached_ = false;
  bool                              preempted_;
  Eigen::Affine3d                   T_b_t_start_;

  Eigen::Affine3d T_b_t_goal_;
  Eigen::Vector6d target_twist_;


  bool        is_relative_;
  std::string goal_link_;
  std::string goal_reference_frame_;

  std::string action_ns_;
  void poseGoalCallback   (actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>::GoalHandle gh);
  void poseCancelCallback (actionlib::ActionServer<cartesian_motion_msgs::cartMotionAction>::GoalHandle gh);
  void poseThread         ( );

  bool stop_thread_;

};
}
}







#endif

