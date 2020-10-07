#include <cnr_open_loop_effort_controller/cnr_open_loop_effort_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::OpenLoopEffortController, controller_interface::ControllerBase)


namespace cnr
{
namespace control
{

bool OpenLoopEffortController::doInit( )
{
  CNR_TRACE_START(*m_logger);
  if (!getControllerNh().getParam("setpoint_topic_name", m_setpoint_topic_name))
  {
    CNR_RETURN_FALSE(*m_logger, getControllerNamespace() + "/'setpoint_topic_name' does not exist");
  }
  add_subscriber<sensor_msgs::JointState>(m_setpoint_topic_name, 1,
                                             boost::bind(&OpenLoopEffortController::callback,this,_1));

  CNR_DEBUG(*m_logger, "Controller ' " + getControllerNamespace() + "' controls the following joint: " + cnr_controller_interface::to_string(jointNames()));
  CNR_DEBUG(*m_logger, "Controller ' " + getControllerNamespace() + "' get the setpoint from the topic: '" + m_setpoint_topic_name + "'");

  std::vector<double> max_effort;
  if (!getControllerNh().getParam("maximum_torque",max_effort))
  {
    CNR_WARN(*m_logger, "no maximum_torque specified");
    m_max_effort.resize(nAx());
    m_max_effort.setZero();
  }
  if(max_effort.size() != nAx() )
  {
    CNR_RETURN_FALSE( *m_logger, "maximum_torque mismatches dimension!");
  }
  m_eff_cmd.resize(nAx());
  m_eff_cmd.setZero();
  m_max_effort.resize(nAx());
  m_max_effort = Eigen::Map<Eigen::VectorXd>(max_effort.data(), nAx());

  this->setPriority(Q_PRIORITY);
  CNR_RETURN_TRUE(*m_logger);
}

bool OpenLoopEffortController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_configured = false;
  CNR_RETURN_TRUE(*m_logger);
}

bool OpenLoopEffortController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_configured = false;
  CNR_RETURN_TRUE(*m_logger);
}

bool OpenLoopEffortController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 10.0);
  try
  {
    for(size_t iAx=0; iAx<nAx(); iAx++)
    if (m_eff_cmd(iAx) > m_max_effort(iAx))
    {
      m_eff_cmd(iAx) = m_max_effort(iAx);
    }
    else if (m_eff_cmd(iAx)<-m_max_effort(iAx))
    {
      m_eff_cmd(iAx) = -m_max_effort(iAx);
    }
    setCommandEffort(m_eff_cmd);
  }
  catch (...)
  {
    CNR_WARN(*m_logger, "something wrong!");
    m_eff_cmd.setZero();
    setCommandEffort( m_eff_cmd );
  }
  CNR_RETURN_TRUE_THROTTLE(*m_logger, 10.0);
}


bool OpenLoopEffortController::extractJoint(const sensor_msgs::JointState& msg)
{
  size_t cnt = 0;
  Eigen::VectorXd target = getCommandEffort();
  for (size_t iJoint=0; iJoint < msg.name.size(); iJoint++)
  {
    for (size_t iAx=0; iAx < jointNames().size(); iAx++)
    {
      if (msg.name.at(iJoint) == jointNames().at(iAx))
      {
        if (msg.effort.size() > (iJoint))
        {
          target(iAx) = msg.effort.at(iJoint);
          cnt++;
        }
        else
        {
          return false;
        }
      }
    }
  }

  bool ok = (cnt == nAx());
  return ok;
}

void OpenLoopEffortController::callback(const boost::shared_ptr<sensor_msgs::JointState const>& msg)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0);
  if (extractJoint(*msg))
  {
    m_configured = true;
  }
  else
  {
    CNR_FATAL(*m_logger, getControllerNamespace() + " command message dimension is wrong.");
  }
  CNR_RETURN_OK_THROTTLE(*m_logger, void(), 5.0);
}



		
}
}
