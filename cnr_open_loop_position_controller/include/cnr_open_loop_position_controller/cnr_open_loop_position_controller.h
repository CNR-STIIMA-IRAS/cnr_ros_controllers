#ifndef cnr_open_loop_position_controller__20190204
#define cnr_open_loop_position_controller__20190204


#include <hardware_interface/joint_command_interface.h>

#include <thread>
#include <mutex>
#include <boost/graph/graph_concepts.hpp>
#include <ros/time.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>

#include <subscription_notifier/subscription_notifier.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>

namespace cnr
{
namespace control
{


class OpenLoopPositionController : public cnr_controller_interface::JointCommandController<hardware_interface::PositionJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

  virtual void startJointKinematicStatus(Eigen::VectorXd& q,
                                         Eigen::VectorXd& qd,
                                         Eigen::VectorXd& qdd,
                                         Eigen::VectorXd& effort );

  virtual void updateJointKinematicStatus(Eigen::VectorXd& q,
                                          Eigen::VectorXd& qd,
                                          Eigen::VectorXd& qdd,
                                          Eigen::VectorXd& effort );


protected:

  std::string setpoint_topic_name;
  bool        m_configured;

  void callback(const sensor_msgs::JointStateConstPtr msg);
  bool extractJoint(const sensor_msgs::JointState &msg);

  const std::string   SP_TOPIC_ID = "sp";
};


}
}

# endif
