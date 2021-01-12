#include <std_msgs/Int8MultiArray.h>
#include <digital_io_controller/digital_io_controller.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE 

PLUGINLIB_EXPORT_CLASS(ros::control::DigitalInputController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(ros::control::DigitalOutputController, controller_interface::ControllerBase)



namespace std
{
inline std::string to_string( const std::vector<std::string>& vals )
{
  std::string ret = "< ";
  for( auto const & val : vals ) ret += val + ", ";
  ret += " >";
  return ret;
}

inline std::string to_string( const std::string& val )
{
  return val;
}

inline std::string to_string( const bool& val )
{
  return val ? "TRUE" : "FALSE";
}

}


#define GET_AND_RETURN( nh, param, value )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_ERROR("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    return false;\
  }



#define GET_AND_DEFAULT( nh, param, value, def )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_WARN("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    ROS_WARN("Default value '%s' superimposed. ", std::to_string( def ).c_str() );\
    value=def;\
  }


namespace ros
{
namespace control
{

bool DigitalInputController::doInit ()
{
  CNR_TRACE_START(this->logger());
  m_di_names.clear();
  GET_AND_RETURN(getControllerNh(), "digital_input_names", m_di_names );
  GET_AND_RETURN(getControllerNh(), "published_topic",m_di_topic_name);

  m_num_di = m_di_names.size();
  m_di_sub_handle = add_publisher<std_msgs::Int8MultiArray>(m_di_topic_name,1);
  CNR_RETURN_TRUE(this->logger());
}


bool DigitalInputController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  std_msgs::Int8MultiArrayPtr msg(new std_msgs::Int8MultiArray());
  msg->data.resize(m_num_di,0);
  msg->layout.dim.resize( m_num_di );
  for (std::size_t idx=0;idx<m_num_di;idx++)
  {
    msg->layout.dim.at(idx).label = m_di_names.at(idx);
    msg->data.at(idx)=m_hw->getHandle(m_di_names.at(idx)).getValue();
  }
  if(!publish(m_di_sub_handle, msg))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}



bool DigitalOutputController::doInit ()
{
  GET_AND_RETURN(getControllerNh(), "digital_output_names", m_do_names);
  GET_AND_RETURN(getControllerNh(), "subscribed_topic", m_do_topic_name);
  m_num_do = m_do_names.size();
  m_do.resize( m_do_names.size(), 0 );
  add_subscriber<std_msgs::Int8MultiArray>(m_do_topic_name, 5,
                                    boost::bind(&DigitalOutputController::callback, this, _1) );
  CNR_RETURN_TRUE(this->logger());
}

void DigitalOutputController::callback(const std_msgs::Int8MultiArrayConstPtr& msg )
{
  for(int i=0; i<std::min( (int)msg->data.size(), (int)m_do.size() );i++)
  {
    m_do.at(i) = msg->data[i];
  }
}

bool DigitalOutputController::doUpdate ( const ros::Time& /*time*/, const ros::Duration& /*period*/ )
{
  for (std::size_t idx=0;idx<m_num_do;idx++)
  {
    m_hw->getHandle(m_do_names.at(idx)).setCommand( m_do.at(idx) );
  }
  CNR_RETURN_TRUE(this->logger());
}



}
}
