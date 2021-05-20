#ifndef ForceTorquePublisher_201809210739
#define ForceTorquePublisher_201809210739

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_hardware_interface/force_torque_state_interface.h>
#include <cnr_hardware_interface/force_torque_command_interface.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <ros/ros.h>

namespace ros
{
namespace control
{

class ForceTorquePublisher: public cnr::control::Controller<hardware_interface::ForceTorqueSensorInterface>
{
public:
  virtual bool doInit( );
  virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);

protected:
  size_t      m_w_sub_handle;
  std::string m_name;
  hardware_interface::ForceTorqueSensorHandle m_ft_handle;
};

}
}


#endif
