#ifndef AnalogStatePublisher_201901210739
#define AnalogStatePublisher_201901210739

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_hardware_interface/analog_state_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>

namespace ros
{
namespace control
{
  
  class AnalogStatePublisher: public cnr::control::Controller<hardware_interface::AnalogStateInterface>
  {
  public:
    virtual bool doInit( );
    virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
    
  protected:
    std::vector<std::string> m_analog_names;
    std::string m_topic_name;
    size_t m_pub_idx;
    
    std::size_t m_nax;
  };
}
}


#endif
