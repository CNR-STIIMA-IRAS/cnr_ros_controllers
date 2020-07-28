#ifndef cnr_cart_teleop_controller__201812051146
#define cnr_cart_teleop_controller__201812051146

#include <ros/ros.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
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


namespace cnr
{
namespace control
{
class CartTeleopController: public cnr_controller_interface::JointController<hardware_interface::PosVelEffJointInterface>
{

public:
  CartTeleopController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:
  std::vector<hardware_interface::PosVelEffJointHandle> m_jh;

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
  geometry_msgs::TwistStamped::Ptr m_twist;

  std::mutex m_mtx;
  void callback(const geometry_msgs::TwistStamped::ConstPtr& msg);

  boost::shared_ptr<rosdyn::Chain> m_chain;

  Eigen::Matrix<double, 6, 1> m_target_twist;



};
}
}







#endif
