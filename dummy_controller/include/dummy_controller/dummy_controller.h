#ifndef __dummy_controller__
#define __dummy_controller__

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace cnr
{
namespace control
{
  
class DummyController: public cnr::control::Controller<hardware_interface::JointStateInterface>
{
public:
  virtual bool doInit();
  virtual bool doStarting(const ros::Time& time);
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
  virtual bool doStopping(const ros::Time& time);

private:
  std::string m_joint_name;
  
  
};

  
}
}

# endif
