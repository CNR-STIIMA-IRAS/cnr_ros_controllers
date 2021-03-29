#include <force_torque_publisher/force_torque_publisher.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE 
PLUGINLIB_EXPORT_CLASS(ros::control::ForceTorquePublisher, controller_interface::ControllerBase)

namespace ros
{
namespace control
{

bool ForceTorquePublisher::doInit()
{
  CNR_TRACE_START(m_logger);
  if (!getControllerNh().getParam("name",m_name))
  {
    CNR_RETURN_FALSE(m_logger, getControllerNamespace() + "/name is not in the rosparam server. Abort. " );
    return false;
  }
  std::string published_topic;
  if (!getControllerNh().getParam("published_topic",published_topic))
  {
    CNR_RETURN_FALSE(m_logger, getControllerNamespace() + "/published_topic is not in the rosparam server. Abort. " );
  }

  m_w_sub_handle = add_publisher<geometry_msgs::WrenchStamped>(published_topic,1);
  CNR_RETURN_TRUE(m_logger);
}

bool ForceTorquePublisher::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  geometry_msgs::WrenchStampedPtr msg(new geometry_msgs::WrenchStamped());

  msg->header.frame_id = m_hw->getHandle( m_name ).getFrameId( );
  msg->wrench.force.x  = m_hw->getHandle( m_name ).getForce()[0];
  msg->wrench.force.y  = m_hw->getHandle( m_name ).getForce()[1];
  msg->wrench.force.z  = m_hw->getHandle( m_name ).getForce()[2];

  msg->wrench.torque.x = m_hw->getHandle( m_name ).getTorque()[0];
  msg->wrench.torque.y = m_hw->getHandle( m_name ).getTorque()[1];
  msg->wrench.torque.z = m_hw->getHandle( m_name ).getTorque()[2];

  msg->header.stamp=ros::Time::now();
  if(!publish(m_w_sub_handle, msg))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  return true;
}


}
}
