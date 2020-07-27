#ifndef cnr_cart_teleop_controller__201812051146
#define cnr_cart_teleop_controller__201812051146

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

#if ROS_VERSION_MINIMUM(1, 14, 1)
# include <memory>
namespace shared_ptr_namespace = std;
#else
# include <boost/concept_check.hpp>
# include <boost/graph/graph_concepts.hpp>
# include <boost/enable_shared_from_this.hpp>
namespace shared_ptr_namespace = boost;
#endif


namespace itia
{
namespace control
{
class CartTeleopController: public controller_interface::Controller<hardware_interface::PosVelEffJointInterface>
{

public:
  CartTeleopController();
  bool init(hardware_interface::PosVelEffJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);

protected:
  hardware_interface::PosVelEffJointInterface* m_hw;
  std::vector<hardware_interface::PosVelEffJointHandle> m_jh;
  ros::NodeHandle m_nh;
  ros::NodeHandle m_root_nh;
  ros::NodeHandle m_controller_nh;
  std::vector<std::string> m_joint_names;
  unsigned int m_nAx;
  std::vector<double> m_target_vel;
  std::vector<double> m_target_pos;
  std::vector<double> m_cmd_pos_old;
  std::vector<double> m_upper_limit;
  std::vector<double> m_lower_limit;
  Eigen::VectorXd m_last_target_vel;
  Eigen::VectorXd m_velocity_limit;
  Eigen::VectorXd m_acceleration_limit;

  double m_max_cart_lin_vel;
  double m_max_cart_lin_acc;

  double m_max_cart_ang_vel;
  double m_max_cart_ang_acc;
  Eigen::Vector6d m_last_twist_of_in_b;

  std::vector<double> m_cmd_pos;
  double m_time;
  bool m_configured;
  urdf::ModelInterfaceSharedPtr m_model;
  double m_err;
  double m_err_old = 0;
  std::shared_ptr<ros_helper::SubscriptionNotifier<sensor_msgs::JointState>> m_joint_target_rec;




  std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::TwistStamped>> m_cart_target_rec;
  boost::shared_ptr<rosdyn::Chain> m_chain;

  Eigen::Matrix<double, 6, 1> m_target_twist;



};
}
}







#endif
