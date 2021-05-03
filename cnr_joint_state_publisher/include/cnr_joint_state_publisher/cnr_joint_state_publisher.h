#ifndef CNR_JOINT_STATE_PUBLISHER__NR_JOINT_STATE_PUBLISHER__H
#define CNR_JOINT_STATE_PUBLISHER__NR_JOINT_STATE_PUBLISHER__H

#include <cnr_controller_interface/cnr_joint_controller_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/JointState.h>

namespace cnr
{
namespace control
{

class JointStatePublisher:
  public cnr::control::JointController<hardware_interface::JointStateHandle, hardware_interface::JointStateInterface>
{
public:
  virtual ~JointStatePublisher();

protected:
  virtual bool doInit();
  virtual bool doStarting(const ros::Time& time);
  virtual bool doUpdate(const ros::Time&, const ros::Duration&);
  virtual bool doStopping(const ros::Time& time);

  size_t m_pub_handle;
};

}  // namespace control
}  // namespace ros

#endif  // CNR_JOINT_STATE_PUBLISHER__NR_JOINT_STATE_PUBLISHER__H
