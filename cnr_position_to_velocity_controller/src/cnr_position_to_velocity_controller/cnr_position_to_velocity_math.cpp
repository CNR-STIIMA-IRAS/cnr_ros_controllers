#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_math.h>
#include <urdf/model.h>
namespace cnr
{
namespace control
{


bool PositionToVelocityControllerMath::init(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("controlled_joint", m_joint_name))
  {
    ROS_ERROR("controlled_joint is not specified");
    return false;
  }

  m_root_nh = root_nh;
  m_controller_nh = controller_nh;
  m_well_init = false;
  m_controller_nh.setCallbackQueue(&m_queue);

  std::string setpoint_topic_name;
  std::string feedforward_topic_name;

  if (!m_controller_nh.getParam("setpoint_topic_name", setpoint_topic_name))
  {
    ROS_ERROR_STREAM(m_controller_nh.getNamespace() + "/'setpoint_topic_name' does not exist");
    ROS_ERROR("ERROR DURING INITIALIZATION CONTROLLER '%s'", m_controller_nh.getNamespace().c_str());
    return false;
  }

  m_target_js_rec.reset(new ros_helper::SubscriptionNotifier<sensor_msgs::JointState>(m_controller_nh, setpoint_topic_name, 1, boost::bind(&PositionToVelocityControllerMath::callback, this, _1)));

  m_use_target_velocity = false;
  if (!m_controller_nh.getParam("use_target_velocity", m_use_target_velocity))
    ROS_DEBUG_STREAM(m_controller_nh.getNamespace() + "/use_target_velocity does not exist, set FALSE");

  m_use_target_torque = false;
  if (!m_controller_nh.getParam("use_target_torque", m_use_target_torque))
    ROS_DEBUG_STREAM(m_controller_nh.getNamespace() + "/use_target_torque does not exist, set FALSE");

  ROS_DEBUG("Controller '%s' controls the following joint: %s", m_controller_nh.getNamespace().c_str(), m_joint_name.c_str());

  m_target_pos_filter.importMatricesFromParam(m_controller_nh, "target_pos_filter");
  m_pos_filter.importMatricesFromParam(m_controller_nh, "pos_filter");
  m_controller.importMatricesFromParam(m_controller_nh, "controller");
  m_integral_controller.importMatricesFromParam(m_controller_nh, "integral_controller");

  if (!m_controller_nh.getParam("position_minimum_error", m_position_minimum_error))
  {
    ROS_WARN("no position_minimum_error specified for joint %s, set equal to zero", m_joint_name.c_str());
    m_position_minimum_error = 0;
  }

  if (!m_controller_nh.getParam("position_maximum_error", m_position_maximum_error))
  {
    ROS_WARN("no position_maximum_error specified for joint %s, set equal to 0.1", m_joint_name.c_str());
    m_position_maximum_error = 0.1;
  }

  if (!m_controller_nh.getParam("antiwindup_ratio", m_antiwindup_gain))
  {
    ROS_WARN("no antiwindup_ratio specified for joint %s, set equal to 1", m_joint_name.c_str());
    m_antiwindup_gain = 1;
  }

  if (!m_controller_nh.getParam("interpolate_setpoint", m_interpolate_setpoint))
  {
    ROS_WARN("interpolate_setpoint specified, set false");
    m_interpolate_setpoint = false;
  }
  else if (!m_controller_nh.getParam("maximum_interpolation_time", m_maximum_interpolation_time))
  {
    ROS_WARN("maximum_interpolation_time specified, set 10 ms");
    m_maximum_interpolation_time = 0.01;
  }

  if (!m_controller_nh.getParam("maximum_velocity", m_max_velocity))
  {
    ROS_INFO("no maximum_velocity specified for joint %s, reading from urdf", m_joint_name.c_str());
    urdf::Model urdf_model;
    if (!urdf_model.initParam("/robot_description"))
    {
      ROS_ERROR("Urdf robot_description '%s' does not exist", (m_controller_nh.getNamespace() + "/robot_description").c_str());
      return false;
    }
    if (urdf_model.getJoint(m_joint_name))
      m_max_velocity = urdf_model.getJoint(m_joint_name)->limits->velocity;
    else
    {
      ROS_ERROR("Joint '%s' is not in the robot_description", m_joint_name.c_str());
      return false;
    }

  }
  ROS_INFO("Controller '%s' well initialized", m_controller_nh.getNamespace().c_str());

  m_well_init = true;

  m_error = false;
  return true;

}

void PositionToVelocityControllerMath::starting(const ros::Time& time, const double& fb_pos, const double& fb_vel)
{
  m_configured = false;

  m_target_pos = fb_pos;
  m_target_vel = 0;
  m_target_eff = 0;
  m_queue.callAvailable();

  ros::Time t0 = ros::Time::now();

  ROS_DEBUG("[ %s ] Creating controller objects", m_controller_nh.getNamespace().c_str());


  Eigen::VectorXd init_pos(1);
  init_pos(0) = fb_pos;
  m_pos_filter.setStateFromLastIO(init_pos, init_pos);


  init_pos(0) = m_target_pos;
  m_target_pos_filter.setStateFromLastIO(init_pos, init_pos);

  Eigen::VectorXd init_vel(1);
  if (m_use_target_velocity)
    init_vel(0) = fb_vel - m_target_vel;
  else
    init_vel(0) = fb_vel;
  Eigen::VectorXd init_error = m_target_pos_filter.getOutput() - m_pos_filter.getOutput();

  m_controller.setStateFromLastIO(init_error, init_vel);
  m_integral_controller.setStateFromLastIO(init_error, init_vel); // TODO fix INITIALIZATION of two controllers
  m_antiwindup = 0;

  ROS_DEBUG("Controller '%s' started in %f seconds", m_controller_nh.getNamespace().c_str(), (ros::Time::now() - t0).toSec());

}

void PositionToVelocityControllerMath::stopping(const ros::Time& time)
{
  ROS_INFO("[ %s ] Stopping controller", m_controller_nh.getNamespace().c_str());
  m_configured = false;
  m_vel_cmd = 0;
  m_eff_cmd = 0;
}

void PositionToVelocityControllerMath::update(const ros::Time& time, const ros::Duration& period, const double& fb_pos, const double& fb_vel)
{
  try
  {
    m_queue.callAvailable();

    if (!m_configured)
    {
      m_target_pos = fb_pos;
      m_target_vel = 0;
    }

    double filter_output;
    double target_filter_output;

    double controller_input;
    double controller_output;
    double integral_controller_input;
    double integral_controller_output;

    filter_output = m_pos_filter.update(fb_pos);

    if (m_configured)
    {

      if (m_interpolate_setpoint && ((time.toSec() - m_last_sp_time) <= m_maximum_interpolation_time))
      {
        m_target_pos += m_target_vel * (time.toSec() - m_last_sp_time);
      }
      target_filter_output = m_target_pos_filter.update(m_target_pos);
    }
    else
      target_filter_output = m_target_pos_filter.update(m_target_pos_filter.getOutput()(0));


    controller_input = target_filter_output - filter_output; //controller error

    if (std::abs(controller_input) > m_position_maximum_error)
      m_error = true;
    else
      m_error = false;

    if (m_error)
    {
      ROS_ERROR("Exceeded the position_maximum_error!");
      m_vel_cmd = 0;
      return;
    }
    integral_controller_input = target_filter_output - filter_output + m_antiwindup_gain * m_antiwindup; //integral controller error

    controller_output = m_controller.update(controller_input);
    integral_controller_output = m_integral_controller.update(integral_controller_input);

    m_pos_cmd = target_filter_output;
    m_vel_cmd = controller_output + integral_controller_output;
    m_eff_cmd = 0;

//        ROS_INFO_THROTTLE(0.1,"r=%f, y=%f, er=%f vel=%f, max_vel=%f",target_filter_output,filter_output,controller_input,m_vel_cmd, m_max_velocity);
    if (m_use_target_velocity)
      m_vel_cmd += m_target_vel;


    if (m_use_target_torque)
      m_eff_cmd = m_target_eff;
    else
      m_eff_cmd = 0;

    if (m_vel_cmd > m_max_velocity)
    {
      m_antiwindup = m_max_velocity - m_vel_cmd;
      m_vel_cmd = m_max_velocity;
    }
    else if (m_vel_cmd < -m_max_velocity)
    {
      m_antiwindup = -m_max_velocity - m_vel_cmd;
      m_vel_cmd = -m_max_velocity;
    }
    else
      m_antiwindup = 0;

//         ROS_INFO("r=%f, y=%f, er=%f vel=%f",target_filter_output,filter_output,controller_input,m_vel_cmd);

  }
  catch (...)
  {
    ROS_WARN("something wrong: Controller '%s'", m_controller_nh.getNamespace().c_str());
    m_vel_cmd = 0;
  }

}

bool PositionToVelocityControllerMath::extractJoint(const sensor_msgs::JointState msg, const std::string name, double& pos, double& vel, double& eff)
{
  for (unsigned int iJoint = 0; iJoint < msg.name.size(); iJoint++)
  {
    if (!msg.name.at(iJoint).compare(name))
    {
      if (msg.position.size() > (iJoint))
        pos = msg.position.at(iJoint);
      else
        return false;

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

void PositionToVelocityControllerMath::callback(const sensor_msgs::JointStateConstPtr msg)
{
  if (extractJoint(*msg, m_joint_name, m_target_pos, m_target_vel, m_target_eff))
  {
    m_configured = true;
    m_last_sp_time = msg->header.stamp.toSec();
  }
  else
  {
    ROS_FATAL_STREAM(m_controller_nh.getNamespace() + " target message dimension is wrong");
    ROS_FATAL_STREAM(m_controller_nh.getNamespace() + " msg received: " << *msg);
  }
  return;
}


}
}
