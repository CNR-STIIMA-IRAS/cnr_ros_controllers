#include <dummy_controller/dummy_controller.h>

#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE 
PLUGINLIB_EXPORT_CLASS(cnr::control::DummyController, controller_interface::ControllerBase)

namespace cnr
{
namespace control
{

bool DummyController::doInit()
{

  if(!getControllerNh().getParam("controlled_joint",m_joint_name))
  {
    ROS_FATAL("ERROR");
    return false;
  }
  bool flag=false;
  for(unsigned idx=0;idx<m_hw->getNames().size();idx++)
  {
    if(!m_hw->getNames().at(idx).compare(m_joint_name))
    {
      m_hw->getHandle(m_joint_name);
      flag=true;
      break;
    }
  }
  if(!flag)
  {
    ROS_FATAL("ERROR");
    return false;
  }
  
  
  return true;
}

bool DummyController::doStarting(const ros::Time& /*time*/)
{
  /* called immediately before starting the control loop */
  return true;
}

bool DummyController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  /* called every iteration when the controller is active*/
  return true;
}

bool DummyController::doStopping(const ros::Time& /*time*/)
{
  /* called before the destructor*/
  return true;
}


  
}
}
