#include <cnr_velocity_to_torque_controller/cnr_velocity_to_torque_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::VelocityToTorqueController, controller_interface::ControllerBase)


namespace cnr
{
namespace control
{

bool VelocityToTorqueController::doInit( )
{
  std::string setpoint_topic_name;

  if (!getControllerNh().getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_FATAL_STREAM(getControllerNamespace() + "/'setpoint_topic_name' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", getControllerNamespace().c_str());
    return false;
  }
  add_subscriber<sensor_msgs::JointState>(setpoint_topic_name,1,
         boost::bind(&VelocityToTorqueController::callback, this, _1));

  m_use_target_torque = false;
  if (!getControllerNh().getParam("use_target_torque", m_use_target_torque))
  {
    ROS_WARN_STREAM(getControllerNamespace() + "/use_target_torque does not exist, set FALSE");
  }

  ROS_DEBUG("Controller '%s' controls the following joint: %s", getControllerNamespace().c_str(), jointNames().front().c_str());


  m_filter.importMatricesFromParam(getControllerNh(), "vel_filter");
  m_target_filter.importMatricesFromParam(getControllerNh(), "target_vel_filter");
  m_controller.importMatricesFromParam(getControllerNh(), "controller");
  m_integral_controller.importMatricesFromParam(getControllerNh(), "integral_controller");

  if (!getControllerNh().getParam("antiwindup_ratio", m_antiwindup_gain))
  {
    ROS_WARN("no antiwindup_gain specified for joint %s, set equal to 1", jointNames().front().c_str());
    m_antiwindup_gain = 1;
  }
  if (!getControllerNh().getParam("maximum_torque", m_max_effort))
  {
    ROS_WARN("no maximum_torque specified for joint %s, set equal to zero", jointNames().front().c_str());
    m_max_effort = 0;
  }
  ROS_INFO("Controller '%s' well initialized", getControllerNamespace().c_str());

  m_well_init = true;

  this->setPriority(QD_PRIORITY);

  return true;
}

bool VelocityToTorqueController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_configured = false;

  double fb_vel = q(0);
  double fb_eff = effort(0);

  m_target_vel = fb_vel;
  m_target_eff = 0;

  ros::Time t0 = ros::Time::now();


  ROS_DEBUG("[ %s ] Creating controller objects",  getControllerNamespace().c_str());

  Eigen::VectorXd init_vel(1);
  init_vel(0) = fb_vel;
  m_filter.setStateFromLastIO(init_vel, init_vel);

  init_vel(0) = m_target_vel;
  m_target_filter.setStateFromLastIO(init_vel, init_vel);

  Eigen::VectorXd init_eff(1);
  if (m_use_target_torque)
    init_eff(0) = fb_eff - m_target_eff;
  else
    init_eff(0) = fb_eff;

  Eigen::VectorXd init_error = m_target_filter.getOutput() - m_filter.getOutput();

  m_controller.setStateFromLastIO(init_error, init_eff);
  m_integral_controller.setStateFromLastIO(init_error, init_eff); // TODO fix INITIALIZATION of two controllers
  m_antiwindup = 0;

  ROS_DEBUG("Controller '%s' started in %f seconds", getControllerNamespace().c_str(), (ros::Time::now() - t0).toSec());
  CNR_RETURN_TRUE(*m_logger);
}

bool VelocityToTorqueController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_configured = false;
  m_vel_cmd = 0;
  CNR_RETURN_TRUE(*m_logger);
}

bool VelocityToTorqueController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  try
  {
    if (!m_configured)
    {
      m_target_vel = qd(0);
      m_target_eff = 0;

    }

    double filter_output;
    double target_filter_output;

    double controller_input;
    double controller_output;
    double integral_controller_input;
    double integral_controller_output;

    filter_output = m_filter.update(qd(0));

    if (m_configured)
      target_filter_output = m_target_filter.update(m_target_vel);
    else
      target_filter_output = m_target_filter.update(m_target_filter.getOutput()(0));

    controller_input = target_filter_output - filter_output; //controller error
    integral_controller_input = target_filter_output - filter_output + m_antiwindup_gain * m_antiwindup; //integral controller error

    controller_output = m_controller.update(controller_input);
    integral_controller_output = m_integral_controller.update(integral_controller_input);

    m_vel_cmd = target_filter_output;
    m_eff_cmd = controller_output + integral_controller_output;

    if (m_use_target_torque)
      m_eff_cmd += m_target_eff;

    if (m_eff_cmd > m_max_effort)
    {
      m_antiwindup = m_max_effort - m_eff_cmd;
      m_eff_cmd = m_max_effort;
    }
    else if (m_eff_cmd < -m_max_effort)
    {
      m_antiwindup = -m_max_effort - m_eff_cmd;
      m_eff_cmd = -m_max_effort;
    }
    else
      m_antiwindup = 0;

    setCommandEffort(m_eff_cmd, 0);
  }
  catch (...)
  {
    ROS_WARN("something wrong: Controller '%s'", getControllerNamespace().c_str());
    setCommandEffort(0, 0);
  }

  CNR_RETURN_TRUE(*m_logger);

}

bool VelocityToTorqueController::extractJoint(const sensor_msgs::JointState msg, const std::string name, double& vel, double& eff)
{
  for (unsigned int iJoint = 0; iJoint < msg.name.size(); iJoint++)
  {
    if (!msg.name.at(iJoint).compare(name))
    {
      if (msg.velocity.size() > (iJoint))
        vel = msg.velocity.at(iJoint);
      else
        return false;

      if (msg.effort.size() > (iJoint))
        eff = msg.effort.at(iJoint);
      else
        return false;

      return true;
    }
  }
  return false;
}

void VelocityToTorqueController::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  if (extractJoint(*msg, jointNames().at(0), m_target_vel, m_target_eff))
  {
    m_configured = true;
  }
  else
  {
    ROS_FATAL_STREAM(getControllerNamespace() + " target message dimension is wrong");
  }
  return;
}


}
}
