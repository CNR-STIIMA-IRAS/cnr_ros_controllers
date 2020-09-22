#include <analog_state_publisher/analog_state_publisher.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE 
PLUGINLIB_EXPORT_CLASS(ros::control::AnalogStatePublisher, controller_interface::ControllerBase);

namespace ros
{
  namespace control
  {
    
    bool AnalogStatePublisher::doInit ( )
    {
      /* called immediately after the constructor */
      if (!getControllerNh().getParam("controlled_joint",m_analog_names))
      {
        ROS_FATAL("ERROR");
        return false;
      }
      if (!getControllerNh().getParam("topic",m_topic_name))
      {
        ROS_FATAL("ERROR");
        return false;
      }

      
      m_nax=m_analog_names.size();
      bool flag=false;
      
      for (std::string name: m_analog_names)
      {
        try 
        {
          auto h = m_hw->getHandle(name);
        }
        catch(...)
        {
          ROS_ERROR("Resource named %s is not managed by hardware_interface",name.c_str());
          ROS_ERROR("Available resources: ");
          for(auto const & n : m_hw->getNames() )
          {
              ROS_ERROR("- %s", n.c_str() );
          }
          return false;
        }
      }
      
      m_pub_idx = add_publisher<std_msgs::Float64MultiArray>(m_topic_name,1);
      return true;
    }
    
    bool AnalogStatePublisher::doStarting( const ros::Time& time )
    {
      
    }
    
    bool AnalogStatePublisher::doUpdate ( const ros::Time& time, const ros::Duration& period )
    {
      std_msgs::Float64MultiArrayPtr msg(new std_msgs::Float64MultiArray());
      msg->data.resize(m_nax,0);
      msg->layout.dim.resize( m_nax );
      
      for (std::size_t idx=0;idx<m_nax;idx++)
      {
        msg->layout.dim.at(idx).label = m_analog_names.at(idx);
        msg->data.at(idx)=m_hw->getHandle(m_analog_names.at(idx)).getValue();
      }
      if(!publish(m_pub_idx, msg))
      {
        CNR_RETURN_FALSE(this->logger());
      }
      CNR_RETURN_TRUE(this->logger());
    }
    
    bool AnalogStatePublisher::doStopping ( const ros::Time& time )
    {
      
      
    }
    
    
    
  }
}
