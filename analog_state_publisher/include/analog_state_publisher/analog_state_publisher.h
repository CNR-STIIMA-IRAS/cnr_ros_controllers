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
  
  class AnalogStatePublisher: public cnr_controller_interface::Controller<hardware_interface::AnalogStateInterface>
  {
  public:
    virtual bool doInit( );
    virtual bool doStarting(const ros::Time& time);
    virtual bool doUpdate(const ros::Time& time, const ros::Duration& period);
    virtual bool doStopping(const ros::Time& time);
    
  protected:
    std::vector<std::string> m_analog_names;
    std::string m_topic_name;
    
    std::size_t m_nax;
  };
}
}


#endif
