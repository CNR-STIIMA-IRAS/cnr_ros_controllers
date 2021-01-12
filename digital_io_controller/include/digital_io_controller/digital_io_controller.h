#ifndef AnalogStatePublisher_201901210739
#define AnalogStatePublisher_201901210739

#include <cnr_controller_interface/cnr_controller_interface.h>
#include <cnr_hardware_interface/digital_state_interface.h>
#include <cnr_hardware_interface/digital_command_interface.h>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>

namespace ros
{
namespace control
{

class DigitalInputController:
    public cnr::control::Controller<hardware_interface::DigitalStateInterface>
{
public:
  virtual bool doInit( );
  virtual bool doUpdate   (const ros::Time& time, const ros::Duration& period);
  virtual bool doStopping (const ros::Time& time);

protected:

  std::vector<hardware_interface::DigitalStateHandle> m_jh;
  std::vector<std::string>                      m_di_names;

  size_t                                        m_di_sub_handle;
  std::string                                   m_di_topic_name;
  std::size_t     m_num_di;
};


class DigitalOutputController:
    public cnr::control::Controller<hardware_interface::DigitalCommandInterface>
{
public:
  virtual bool doInit   ( );
  virtual bool doUpdate (const ros::Time& time, const ros::Duration& period);

protected:
  void callback (const std_msgs::Int8MultiArrayConstPtr& msg );

  std::vector<std::string> m_do_names;
  std::vector<int>         m_do;
  std::string              m_do_topic_name;
  std::size_t              m_num_do;
};

}
}


#endif
