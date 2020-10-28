#include <cnr_joint_state_publisher/cnr_joint_state_publisher.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointStatePublisher, controller_interface::ControllerBase)


namespace cnr
{
namespace control
{

JointStatePublisher::~JointStatePublisher()
{
  CNR_TRACE_START(*m_logger);
  if (!isStopped())
  {
    stopping(ros::Time::now());
  }
}

bool JointStatePublisher::doInit()
{
  CNR_TRACE_START(*m_logger);
  if (nAx() == 0)
  {
    CNR_RETURN_FALSE(*m_logger, "The number of controlled axes is 0. Check the configuration. Abort");
  }
  m_pub_handle = this->add_publisher<sensor_msgs::JointState>("joint_states", 1);

  m_msg.reset(new sensor_msgs::JointState());
  m_msg->position.resize(nAx(), 0);
  m_msg->velocity.resize(nAx(), 0);
  m_msg->effort.resize(nAx(), 0);
  m_msg->name = jointNames();

  CNR_TRACE(*m_logger, "Published Topic '" + getPublisher(m_pub_handle)->getTopic()
                     + "', axis names: " + cnr_controller_interface::to_string(jointNames())
            + " n. axes: " + std::to_string(nAx()));
  CNR_RETURN_TRUE(*m_logger);
}

bool JointStatePublisher::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  CNR_RETURN_TRUE(*m_logger);
}

bool JointStatePublisher::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(*m_logger,);
  try
  {
    for(std::size_t iAx = 0; iAx<nAx(); iAx++)
    {
      m_msg->name    .at(iAx) = jointName(iAx);
      m_msg->position.at(iAx) = q(iAx);
      m_msg->velocity.at(iAx) = qd(iAx);
      m_msg->effort  .at(iAx) = effort(iAx);
    }
    m_msg->header.stamp = ros::Time::now();
    if(!publish(m_pub_handle, *m_msg))
    {
      CNR_RETURN_FALSE(this->logger());
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(*m_logger, "Exception caught" + std::string(e.what()));
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(*m_logger);
}

bool JointStatePublisher::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  getPublisher(m_pub_handle)->shutdown();
  CNR_RETURN_TRUE(*m_logger);
}


}
}
