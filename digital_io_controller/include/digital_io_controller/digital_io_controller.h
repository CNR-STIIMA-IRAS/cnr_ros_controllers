#ifndef AnalogStatePublisher_201901210739
#define AnalogStatePublisher_201901210739

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_hardware_interface/digital_state_interface.h>
#include <cnr_hardware_interface/digital_command_interface.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace ros
{
namespace control
{
  
  class DigitalInputController:
      public cnr_controller_interface::Controller<hardware_interface::DigitalStateInterface>
  {
  public:
    virtual bool doInit( );
    virtual bool doUpdate   (const ros::Time& time, const ros::Duration& period);
    virtual bool doStopping (const ros::Time& time);
    
  protected:

    std::vector<hardware_interface::DigitalStateHandle> m_jh;
    std::vector<std::string>                      m_di_names;

    std::string                                   m_di_topic_name;
    std::size_t     m_num_di;
  };


  class DigitalOutputController:
      public cnr_controller_interface::Controller<hardware_interface::DigitalCommandInterface>
  {
  public:
    virtual bool doInit   ( );
    virtual bool doUpdate (const ros::Time& time, const ros::Duration& period);

  protected:
    void callback (std_msgs::Int8MultiArrayConstPtr msg );

    std::vector<std::string> m_do_names;
    std::vector<int>         m_do;
    std::string              m_do_topic_name;
    std::size_t              m_num_do;
  };

}
}


#endif
