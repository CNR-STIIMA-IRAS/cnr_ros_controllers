#include <cnr_joint_state_publisher/cnr_joint_state_publisher.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointStatePublisher, controller_interface::ControllerBase);


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
  if (m_nAx == 0)
  {
    CNR_RETURN_FALSE(*m_logger, "The number of controlled axes is 0. Check the configuration. Abort");
  }
  add_publisher<sensor_msgs::JointState>("js", "joint_states", 1);

  m_msg.reset(new sensor_msgs::JointState());
  m_msg->position.resize(m_nAx, 0);
  m_msg->velocity.resize(m_nAx, 0);
  m_msg->effort.resize(m_nAx, 0);
  m_msg->name = m_joint_names;

  CNR_TRACE(*m_logger, "Published Topic '" + getPublisher("js")->getTopic() + "', axis names: " + cnr_controller_interface::to_string(m_joint_names) + " n. axes: " + std::to_string(m_nAx));
  CNR_RETURN_TRUE(*m_logger);
}

bool JointStatePublisher::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  CNR_RETURN_TRUE(*m_logger);
}

bool JointStatePublisher::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{

  CNR_TRACE_START_THROTTLE(*m_logger, 10.0);
  try
  {
    sensor_msgs::JointStatePtr msg(new sensor_msgs::JointState());

    for (std::size_t idx = 0; idx < m_nAx; idx++)
    {
      msg->name    .push_back(m_joint_names.at(idx));
      msg->position.push_back(m_hw->getHandle(m_joint_names.at(idx)).getPosition());
      msg->velocity.push_back(m_hw->getHandle(m_joint_names.at(idx)).getVelocity());
      msg->effort  .push_back(m_hw->getHandle(m_joint_names.at(idx)).getEffort());
    }
    msg->header.stamp = ros::Time::now();
    publish("js", *msg);
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(*m_logger, "Exception caught" + std::string(e.what()));
  }
  CNR_RETURN_TRUE_THROTTLE(*m_logger, 10.0);
}

bool JointStatePublisher::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  getPublisher("js")->shutdown();
  CNR_RETURN_TRUE(*m_logger);
}


}
}
