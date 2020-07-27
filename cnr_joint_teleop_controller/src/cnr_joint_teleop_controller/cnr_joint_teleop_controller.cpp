#include <cnr_joint_teleop_controller/cnr_joint_teleop_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointTeleopController, controller_interface::ControllerBase);


namespace cnr
{
namespace control
{
JointTeleopController::JointTeleopController()
{
}


bool JointTeleopController::doInit()
{

  if (!getControllerNh().getParam("controlled_joints", m_joint_names))
  {
    ROS_FATAL_STREAM(getControllerNamespace() + "/'controlled_joints' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", getControllerNamespace().c_str());
    return false;
  }

  m_jh.resize(m_nAx);
  m_target_vel.resize(m_nAx);
  m_last_target_vel.resize(m_nAx);
  m_cmd_pos.resize(m_nAx);
  m_err_pos_delta.resize(m_nAx);
  m_saturated_vel.resize(m_nAx);
  for (unsigned int iAx = 0; iAx < m_nAx; iAx++)
  {
    m_jh.at(iAx) = m_hw->getHandle(m_joint_names.at(iAx));
  }


  //INIT PUB/SUB
  std::string setpoint_topic_name;
  setpoint_topic_name = getControllerNamespace() + "/target_joint_teleop";

  add_subscriber("joint_target", setpoint_topic_name, 5, &JointTeleopController::callback, this);

  ROS_INFO("JointTeleopController '%s' well initialized", getControllerNamespace().c_str());
  return true;
}


bool JointTeleopController::doStarting(const ros::Time& time)
{
  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {
    m_target_vel.at(idx) = 0.0;
    m_last_target_vel.at(idx) = 0.0;
    m_cmd_pos.at(idx) = m_jh.at(idx).getPosition();
  }
  m_check1 = false;
  return true;
}



bool JointTeleopController::doStopping(const ros::Time& time)
{
  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {
    m_jh.at(idx).setCommandPosition(m_cmd_pos.at(idx)); //Set as command the last m_cmd_pos
  }
  return true;
}



bool JointTeleopController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  m_time = period.toSec();
  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {

    //Acceleration limits
    m_saturated_vel.at(idx) = std::max(m_target_vel.at(idx), m_last_target_vel.at(idx) - m_acceleration_limit.at(idx) * m_time);
    m_saturated_vel.at(idx) = std::min(m_saturated_vel.at(idx), m_last_target_vel.at(idx) + m_acceleration_limit.at(idx) * m_time);


    //Computing breaking distance
    double t_break = std::abs(m_saturated_vel.at(idx)) / m_acceleration_limit.at(idx);
    double breaking_distance = 0.5 * m_acceleration_limit.at(idx) * std::pow(t_break, 2.0);

    if (m_jh.at(idx).getPosition() > (m_upper_limit.at(idx) - breaking_distance))
    {
      ROS_WARN_THROTTLE(2, "Breaking, maximum limit approaching on joint %s", m_joint_names.at(idx).c_str());
      if (m_saturated_vel.at(idx) > 0)
        m_saturated_vel.at(idx) = std::max(0.0, m_last_target_vel.at(idx) - m_acceleration_limit.at(idx) * m_time);
    }

    if (m_jh.at(idx).getPosition() < (m_lower_limit.at(idx) + breaking_distance))
    {
      ROS_WARN_THROTTLE(2, "Breaking, minimum limit approaching on joint %s", m_joint_names.at(idx).c_str());
      if (m_saturated_vel.at(idx) < 0)
        m_saturated_vel.at(idx) = std::min(0.0, m_last_target_vel.at(idx) + m_acceleration_limit.at(idx) * m_time);
    }
  }

  double saturated_vel_min = *std::min_element(m_saturated_vel.begin(), m_saturated_vel.end());


  //Check used for TeachTeleop and JointSelectPoseTeleop
  if (!m_target_pos.empty())
  {
    if (!m_check1)
      delta();

    for (unsigned int idx = 0; idx < m_nAx; idx++)
    {
      m_err = m_target_pos.at(idx) - m_jh.at(idx).getPosition();
      m_saturated_vel.at(idx) = m_saturated_vel.at(idx) * (abs(m_err) / m_err); //Velocity with correct orientation

      if (!m_check2)
        m_saturated_vel.at(idx) = saturated_vel_min * (abs(m_err) / m_err) * abs(m_err_pos_delta.at(idx) / m_err_delta); ////Velocity with minimum value, correct orientation, scaled

      if (m_err / abs(m_err) != m_err_pos_delta.at(idx) / abs(m_err_pos_delta.at(idx))) //Stop at desired point
        m_saturated_vel.at(idx) = 0.0;
    }
  }


  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {

    m_last_target_vel.at(idx) = m_saturated_vel.at(idx);
    m_cmd_pos.at(idx) = (m_cmd_pos.at(idx) + m_saturated_vel.at(idx) * m_time); //p(k)=p(k+1)+v*t;
    m_jh.at(idx).setCommandPosition(m_cmd_pos.at(idx));
    m_jh.at(idx).setCommandVelocity(m_saturated_vel.at(idx));
  }

  return true;
}


//Used to evaluate inital position error for scale velocity
void JointTeleopController::delta()
{
  std::vector<double> m_err_pos_delta_abs;
  m_err_pos_delta_abs.resize(m_nAx);
  for (unsigned int idx2 = 0; idx2 < m_nAx; idx2++)
  {
    m_err_pos_delta.at(idx2) = m_target_pos.at(idx2) - m_jh.at(idx2).getPosition();
    m_err_pos_delta_abs.at(idx2) = abs(m_err_pos_delta.at(idx2));
  }

  m_err_delta = *std::max_element(m_err_pos_delta_abs.begin(), m_err_pos_delta_abs.end());

  m_check1 = true;
}


void JointTeleopController::callback(const sensor_msgs::JointStateConstPtr msg)
{

  bool check_3 = false;
  int index;
  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {
    if ((msg->name.size() > 1))
      index = idx;
    else
      index = 0;

    if (!m_joint_names.at(idx).compare(msg->name.at(index)))
    {
      m_target_vel.at(idx) = std::min(m_velocity_limit.at(index), std::max(-m_velocity_limit.at(index), msg->velocity.at(index)));

      if (!msg->position.empty())  //Check used for TeachTeleop and JointSelectPoseTeleop
      {
        m_target_pos.resize(m_nAx);
        m_target_pos.at(idx) = msg->position.at(index);
        if (msg->position.size() == 1)
          m_check2 = true; //JointSelectPoseTeleop
        else
          m_check2 = false; //Teach Teleop
      }
      else
      {
        m_target_pos.resize(m_nAx);
        m_target_pos.clear();
        m_check1 = false;
      }
      check_3 = true;
    }
  }

  if (!check_3)
  {
    ROS_WARN("No joint named: %s specified in %s/controlled_joints", msg->name.at(index).c_str(), getControllerNamespace().c_str());
    return;
  }

}

}
}

