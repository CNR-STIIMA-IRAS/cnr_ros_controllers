#ifndef CNR_OPEN_LOOP_POSITION_CONTROLLER__CNR_OPEN_LOOP_POSITION_CONTROLLER__H
#define CNR_OPEN_LOOP_POSITION_CONTROLLER__CNR_OPEN_LOOP_POSITION_CONTROLLER__H


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

template<int N, int MaxN=N>
class OpenLoopPositionControllerN :
    public cnr::control::JointCommandController<N, MaxN, hardware_interface::JointHandle,
                                                            hardware_interface::PositionJointInterface>
{
public:
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:

  std::string m_setpoint_topic_name;
  bool        m_configured;

  void callback(const sensor_msgs::JointStateConstPtr& msg);
  bool extractJoint(const sensor_msgs::JointState& msg);

  const std::string   SP_TOPIC_ID = "sp";
};

using OpenLoopPositionController  = OpenLoopPositionControllerN<-1, cnr::control::max_num_axes>;
using OpenLoopPositionController1 = OpenLoopPositionControllerN<1>;
using OpenLoopPositionController3 = OpenLoopPositionControllerN<3>;
using OpenLoopPositionController6 = OpenLoopPositionControllerN<6>;
using OpenLoopPositionController7 = OpenLoopPositionControllerN<7>;
using OpenLoopPositionController8 = OpenLoopPositionControllerN<8>;

}  // namespace control
}  // namespace cnr

#include <cnr_open_loop_position_controller/internal/cnr_open_loop_position_controller_impl.h>

# endif  // CNR_OPEN_LOOP_POSITION_CONTROLLER__CNR_OPEN_LOOP_POSITION_CONTROLLER__H
