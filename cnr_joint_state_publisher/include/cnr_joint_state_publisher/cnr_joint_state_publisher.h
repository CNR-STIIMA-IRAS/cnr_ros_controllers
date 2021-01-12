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

template<int N,int MaxN=N>
class JointStatePublisherN:
  public cnr::control::JointController<N,MaxN,
      hardware_interface::JointStateHandle, hardware_interface::JointStateInterface>
{
public:

  ~JointStatePublisherN();

protected:
  virtual bool doInit();
  virtual bool doStarting(const ros::Time& time);
  virtual bool doUpdate(const ros::Time&, const ros::Duration&);
  virtual bool doStopping(const ros::Time& time);

  size_t m_pub_handle;
  sensor_msgs::JointStatePtr  m_msg;

};


using JointStatePublisher  = JointStatePublisherN<-1, cnr::control::max_num_axes>;
using JointStatePublisher1 = JointStatePublisherN<1>;
using JointStatePublisher3 = JointStatePublisherN<3>;
using JointStatePublisher6 = JointStatePublisherN<6>;
using JointStatePublisher7 = JointStatePublisherN<7>;

}  // namespace control
}  // namespace ros

#include <cnr_joint_state_publisher/internal/cnr_joint_state_publisher_impl.h>

#endif  // CNR_JOINT_STATE_PUBLISHER__NR_JOINT_STATE_PUBLISHER__H
