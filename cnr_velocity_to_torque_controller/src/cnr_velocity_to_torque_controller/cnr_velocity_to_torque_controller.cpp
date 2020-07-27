#include <cnr_velocity_to_torque_controller/cnr_velocity_to_torque_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(itia::control::VelocityToTorqueController, controller_interface::ControllerBase);


namespace itia
{
namespace control
{

bool VelocityToTorqueController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  m_hw = hw;
  if (!controller_nh.getParam("controlled_joint", m_joint_name))
  {
    ROS_FATAL("ERROR");
    return false;
  }
  bool flag = false;
  for (unsigned idx = 0; idx < m_hw->getNames().size(); idx++)
  {
    if (!m_hw->getNames().at(idx).compare(m_joint_name))
    {
      m_jh = m_hw->getHandle(m_joint_name);
      flag = true;
      break;
    }
  }
  if (!flag)
  {
    ROS_FATAL("ERROR");
    return false;
  }

  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_well_init = false;
  m_controller_nh.setCallbackQueue(&m_queue);

  std::string setpoint_topic_name;

  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace() + "/'setpoint_topic_name' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }
  m_target_js_rec.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(m_controller_nh, setpoint_topic_name, 1, boost::bind(&VelocityToTorqueController::callback, this, _1)));

  m_use_target_torque = false;
  if (!m_controller_nh.getParam("use_target_torque", m_use_target_torque))
  {
    ROS_WARN_STREAM(m_controller_nh.getNamespace() + "/use_target_torque does not exist, set FALSE");
  }

  ROS_DEBUG("Controller '%s' controls the following joint: %s", m_controller_nh.getNamespace().c_str(), m_joint_name.c_str());


  m_filter.importMatricesFromParam(m_controller_nh, "vel_filter");
  m_target_filter.importMatricesFromParam(m_controller_nh, "target_vel_filter");
  m_controller.importMatricesFromParam(m_controller_nh, "controller");
  m_integral_controller.importMatricesFromParam(m_controller_nh, "integral_controller");



  if (!m_controller_nh.getParam("antiwindup_ratio", m_antiwindup_gain))
  {
    ROS_WARN("no antiwindup_gain specified for joint %s, set equal to 1", m_joint_name.c_str());
    m_antiwindup_gain = 1;
  }
  if (!m_controller_nh.getParam("maximum_torque", m_max_effort))
  {
    ROS_WARN("no maximum_torque specified for joint %s, set equal to zero", m_joint_name.c_str());
    m_max_effort = 0;
  }
  ROS_INFO("Controller '%s' well initialized", m_controller_nh.getNamespace().c_str());

  m_well_init = true;
  return true;
}

void VelocityToTorqueController::starting(const ros::Time& time)
{
  m_configured = false;

  double fb_vel = m_jh.getVelocity();
  double fb_eff = m_jh.getEffort();

  m_target_vel = fb_vel;
  m_target_eff = 0;
  m_queue.callAvailable();

  ros::Time t0 = ros::Time::now();


  ROS_DEBUG("[ %s ] Creating controller objects",  m_controller_nh.getNamespace().c_str());

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

  ROS_DEBUG("Controller '%s' started in %f seconds", m_controller_nh.getNamespace().c_str(), (ros::Time::now() - t0).toSec());

}

void VelocityToTorqueController::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s ] Stopping controller",  m_controller_nh.getNamespace().c_str());
  m_configured = false;
  m_vel_cmd = 0;
}

void VelocityToTorqueController::update(const ros::Time& time, const ros::Duration& period)
{
  try
  {
    m_queue.callAvailable();

    double fb_vel = m_jh.getVelocity();
    double fb_eff = m_jh.getEffort();

    if (!m_configured)
    {
      m_target_vel = fb_vel;
      m_target_eff = 0;

    }

    double filter_output;
    double target_filter_output;

    double controller_input;
    double controller_output;
    double integral_controller_input;
    double integral_controller_output;

    filter_output = m_filter.update(fb_vel);

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

    m_jh.setCommand(m_eff_cmd);
  }
  catch (...)
  {
    ROS_WARN("something wrong: Controller '%s'", m_controller_nh.getNamespace().c_str());
    m_jh.setCommand(0);
  }

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

void VelocityToTorqueController::callback(const sensor_msgs::JointStateConstPtr msg)
{
  if (extractJoint(*msg, m_joint_name, m_target_vel, m_target_eff))
  {
    m_configured = true;
  }
  else
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace() + " target message dimension is wrong");
  }
  return;
}


}
}
