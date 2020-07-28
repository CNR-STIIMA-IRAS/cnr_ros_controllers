#include <controller_interface/controller.h>
#include <cnr_cart_teleop_controller/cnr_cart_teleop_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::CartTeleopController, controller_interface::ControllerBase);


namespace cnr
{
namespace control
{
CartTeleopController::CartTeleopController() : m_twist(nullptr)
{
}


bool CartTeleopController::doInit()
{
  if (!getControllerNh().getParam("controlled_joints", m_joint_names))
  {
    ROS_FATAL_STREAM(getControllerNamespace() + "/'controlled_joints' does not exist");
    ROS_FATAL("ERROR DURING INITIALIZATION CONTROLLER '%s'", getControllerNamespace().c_str());
    return false;
  }
  m_nAx = m_joint_names.size();

  std::string robot_description;
  if (!getControllerNh().getParam("/robot_description", robot_description))
  {
    ROS_FATAL_STREAM("Parameter '/robot_description' does not exist");
    return false;
  }
  m_model = urdf::parseURDF(robot_description);

  std::string base_link;
  std::string tool_link; //"ee_link"; "upper_arm_link"  "forearm_link"
  //  std::string model_name=m_model->getName();

  if (!getControllerNh().getParam("base_link", base_link))
  {
    ROS_ERROR("%s/base_link not defined", getControllerNamespace().c_str());
    return false;
  }
  if (!getControllerNh().getParam("tool_link", tool_link))
  {
    ROS_ERROR("%s/tool_link not defined", getControllerNamespace().c_str());
    return false;
  }

  if (!getControllerNh().getParam("max_cartesian_linear_speed", m_max_cart_lin_vel))
  {
    ROS_INFO("%s/max_cartesian_linear_speed not defined, using 0.25 m/s", getControllerNamespace().c_str());
    m_max_cart_lin_vel = 0.25;
  }

  if (!getControllerNh().getParam("max_cartesian_linear_acceleration", m_max_cart_lin_acc))
  {
    ROS_INFO("%s/max_cartesian_linear_acceleration not defined, using 0.75 m/s^2", getControllerNamespace().c_str());
    m_max_cart_lin_acc = 0.75;
  }

  if (!getControllerNh().getParam("max_cartesian_angular_speed", m_max_cart_ang_vel))
  {
    ROS_INFO("%s/max_cartesian_angular_speed not defined, using 0.5 rad/s", getControllerNamespace().c_str());
    m_max_cart_ang_vel = 0.5;
  }

  if (!getControllerNh().getParam("max_cartesian_angular_acceleration", m_max_cart_ang_acc))
  {
    ROS_INFO("%s/max_cartesian_angular_acceleration not defined, using 1.5 rad/s^2", getControllerNamespace().c_str());
    m_max_cart_ang_acc = 1.5;
  }


  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;


  shared_ptr_namespace::shared_ptr<rosdyn::Link> root_link(new rosdyn::Link);  //link primitivo da cui parte la catena cinematica (world ad esempio)
  root_link->fromUrdf(m_model->root_link_);

  m_chain.reset(new rosdyn::Chain(root_link, base_link, tool_link, gravity)); //ricostruisce tutta la catena cinematica andando a leggere l'URDF
  m_chain->setInputJointsName(m_joint_names);


  m_jh.resize(m_nAx);
  m_cmd_pos.resize(m_nAx);
  m_upper_limit.resize(m_nAx);
  m_lower_limit.resize(m_nAx);
  m_velocity_limit.resize(m_nAx);
  m_acceleration_limit.resize(m_nAx);
  m_last_target_vel.resize(m_nAx);
  m_last_target_vel.setZero();
  m_target_vel.resize(m_nAx, 0);

  m_last_twist_of_in_b.setZero();

  for (unsigned int iAx = 0; iAx < m_nAx; iAx++)
  {
    m_jh.at(iAx) = m_hw->getHandle(m_joint_names.at(iAx));
    m_upper_limit.at(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->upper;
    m_lower_limit.at(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->lower;

    if ((m_upper_limit.at(iAx) == 0) && (m_lower_limit.at(iAx) == 0))
    {
      m_upper_limit.at(iAx) = std::numeric_limits<double>::infinity();
      m_lower_limit.at(iAx) = -std::numeric_limits<double>::infinity();
      ROS_INFO("upper and lower limits are both equal to 0, set +/- infinity");
    }

    bool has_velocity_limits;
    if (!getControllerNh().getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/has_velocity_limits", has_velocity_limits))
      has_velocity_limits = false;
    bool has_acceleration_limits;
    if (!getControllerNh().getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/has_acceleration_limits", has_acceleration_limits))
      has_acceleration_limits = false;

    m_velocity_limit(iAx) = m_model->getJoint(m_joint_names.at(iAx))->limits->velocity;
    if (has_velocity_limits)
    {
      double vel;
      if (!getControllerNh().getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity", vel))
      {
        ROS_ERROR_STREAM("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_velocity is not defined");
        return false;
      }
      if (vel < m_velocity_limit(iAx))
        m_velocity_limit(iAx) = vel;
    }

    if (has_acceleration_limits)
    {
      double acc;
      if (!getControllerNh().getParam("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration", acc))
      {
        ROS_ERROR_STREAM("/robot_description_planning/joint_limits/" + m_joint_names.at(iAx) + "/max_acceleration is not defined");
        return false;
      }
      m_acceleration_limit(iAx) = acc;
    }
    else
      m_acceleration_limit(iAx) = 10 * m_velocity_limit(iAx);
  }


  std::string setpoint_topic_name;
  setpoint_topic_name = getControllerNamespace() + "/target_cart_teleop";

  add_subscriber("cart_target", setpoint_topic_name, 1, &CartTeleopController::callback, this);
  return true;
}


bool CartTeleopController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {
    m_cmd_pos.at(idx) = m_jh.at(idx).getPosition();
  }
  CNR_RETURN_TRUE(*m_logger, "Starting controller done");
}



bool CartTeleopController::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  for (unsigned int idx = 0; idx < m_nAx; idx++)
  {
    m_jh.at(idx).setCommandPosition(m_cmd_pos.at(idx)); //Set as command the last m_cmd_pos
  }
  CNR_RETURN_TRUE(*m_logger, "Stopping controller done");
}



bool CartTeleopController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE(*m_logger, 5.0);
  Eigen::Matrix<double, 6, 1> twist;
  twist.setZero();

  if( m_twist )
  {
    twist(0, 0) = m_twist->twist.linear.x;
    twist(1, 0) = m_twist->twist.linear.y;
    twist(2, 0) = m_twist->twist.linear.z;
    twist(3, 0) = m_twist->twist.angular.x;
    twist(4, 0) = m_twist->twist.angular.y;
    twist(5, 0) = m_twist->twist.angular.z;

    Eigen::VectorXd q(m_nAx);

    for (unsigned int idx = 0; idx < m_nAx; idx++)
    {
      q(idx) = m_jh.at(idx).getPosition();
    }

    Eigen::Vector6d twist_of_t_in_b;
    if (!m_twist->header.frame_id.compare("TOOL"))
    {
      Eigen::Affine3d Tbt = m_chain->getTransformation(q);
      twist_of_t_in_b = rosdyn::spatialRotation(twist, Tbt.rotation());
    }
    else if (!m_twist->header.frame_id.compare("BASE"))
    {
      twist_of_t_in_b = twist;
    }
    else
    {
      twist_of_t_in_b = twist;
      ROS_DEBUG("No frame_id defined, assuming base");
    }

    if (twist_of_t_in_b.block(0, 0, 3, 1).norm() > m_max_cart_lin_vel)
      twist_of_t_in_b *= m_max_cart_lin_vel / twist_of_t_in_b.norm();
    if (twist_of_t_in_b.block(3, 0, 3, 1).norm() > m_max_cart_ang_vel)
      twist_of_t_in_b *= m_max_cart_ang_vel / twist_of_t_in_b.norm();


    Eigen::Vector6d Dtwist_of_t_in_b = (twist_of_t_in_b - m_last_twist_of_in_b) / m_time;
    if (Dtwist_of_t_in_b.block(0, 0, 3, 1).norm() > m_max_cart_lin_acc)
      Dtwist_of_t_in_b *= m_max_cart_lin_acc / Dtwist_of_t_in_b.norm();
    if (Dtwist_of_t_in_b.block(3, 0, 3, 1).norm() > m_max_cart_ang_acc)
      Dtwist_of_t_in_b *= m_max_cart_ang_acc / Dtwist_of_t_in_b.norm();

    twist_of_t_in_b = m_last_twist_of_in_b + Dtwist_of_t_in_b * m_time;

    m_last_twist_of_in_b = twist_of_t_in_b;

    Eigen::VectorXd target_vel(m_nAx);
    Eigen::Matrix6Xd J_of_t_in_b;

    J_of_t_in_b = m_chain->getJacobian(q);

    Eigen::FullPivLU<Eigen::MatrixXd> pinv_J(J_of_t_in_b);

    pinv_J.setThreshold(1e-2);
    target_vel = pinv_J.solve(twist_of_t_in_b);
    if (pinv_J.rank() < 6)
    {
      Eigen::MatrixXd null = pinv_J.kernel();
      ROS_WARN_THROTTLE(2, "Singolarity point!");

      for (int iC = 0; iC < null.cols(); iC++)
      {
        Eigen::VectorXd null_versor = null.col(iC);
        null_versor.normalize();
        target_vel = target_vel - (target_vel.dot(null_versor)) * null_versor;
      }
    }

    double ratio;
    ratio = std::abs(target_vel(0)) / m_velocity_limit(0);
    for (unsigned int idx = 1; idx < m_nAx; idx++)
      ratio = std::max(ratio, std::abs(target_vel(idx)) / m_velocity_limit(idx));

    if (ratio > 1)
      target_vel = target_vel / ratio;


    m_time = period.toSec();

    Eigen::VectorXd saturated_vel(m_nAx);
    for (unsigned int idx = 0; idx < m_nAx; idx++)
    {

      // acceleration limit
      saturated_vel(idx) = std::max(target_vel(idx), m_last_target_vel(idx) - m_acceleration_limit(idx) * m_time);
      saturated_vel(idx) = std::min(saturated_vel(idx), m_last_target_vel(idx) + m_acceleration_limit(idx) * m_time);


      // computing breacking distance
      double t_break = std::abs(saturated_vel(idx)) / m_acceleration_limit(idx);
      double breaking_distance = 0.5 * m_acceleration_limit(idx) * std::pow(t_break, 2.0);

      if (m_jh.at(idx).getPosition() > (m_upper_limit.at(idx) - breaking_distance))
      {
        if (saturated_vel(idx) > 0)
        {
          ROS_WARN_THROTTLE(2, "Breaking, maximum limit approaching on joint %s", m_joint_names.at(idx).c_str());
          saturated_vel(idx) = std::max(0.0, m_last_target_vel(idx) - m_acceleration_limit(idx) * m_time);
        }
      }

      if (m_jh.at(idx).getPosition() < (m_lower_limit.at(idx) + breaking_distance))
      {
        ROS_WARN_THROTTLE(2, "Breaking, minimum limit approaching on joint %s", m_joint_names.at(idx).c_str());
        if (saturated_vel(idx) < 0)
          saturated_vel(idx) = std::min(0.0, m_last_target_vel(idx) + m_acceleration_limit(idx) * m_time);
      }
    }

    saturated_vel = saturated_vel.dot(target_vel.normalized()) * target_vel.normalized();
    for (unsigned int idx = 0; idx < m_nAx; idx++)
    {
      m_cmd_pos.at(idx) = m_cmd_pos.at(idx) + saturated_vel(idx) * m_time; //p(k)=p(k+1)+v*t;
      m_last_target_vel(idx) = saturated_vel(idx);
      m_jh.at(idx).setCommandPosition(m_cmd_pos.at(idx));
    }
  }

  CNR_RETURN_TRUE_THROTTLE(*m_logger, 5.0);
}

void CartTeleopController::callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  tick("cart_target");
  std::lock_guard<std::mutex> lock(m_mtx);
  if( m_twist == nullptr )
    m_twist.reset( new geometry_msgs::TwistStamped() );

  *m_twist = *msg;
}

}
}
