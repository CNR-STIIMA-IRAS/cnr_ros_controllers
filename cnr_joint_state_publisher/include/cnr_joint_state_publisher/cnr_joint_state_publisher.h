#ifndef JointStatePublisher_201809210739
#define JointStatePublisher_201809210739

#include <ros/callback_queue.h>
#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace cnr
{
namespace control
{

class JointStatePublisher: public cnr_controller_interface::JointController<hardware_interface::JointStateInterface>
{
public:

  ~JointStatePublisher();

protected:
  virtual bool doInit();
  virtual bool doStarting(const ros::Time& time);
  virtual bool doUpdate(const ros::Time&, const ros::Duration&);
  virtual bool doStopping(const ros::Time& time);

  sensor_msgs::JointStatePtr  m_msg;

};
}
}


#endif
