#include <robot_state_controller/robot_state_controller.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE
PLUGINLIB_EXPORT_CLASS(robot_state_controller::RobotStateController, controller_interface::ControllerBase)

namespace robot_state_controller
{

bool RobotStateController::doInit( )
{

  if (!getControllerNh().getParam("frames",m_frames))
  {
    CNR_RETURN_FALSE(*m_logger, "Param 'frames' not defined");
  }

  std::vector<std::string> link_names=m_kin->linkNames();

  for (unsigned int idx=0;idx<m_frames.size();idx++)
  {
    bool found=false;
    for (unsigned int il=0;il<link_names.size();il++)
    {
      if (!m_frames.at(idx).compare(link_names.at(il)))
      {
        m_frame_idxs.push_back(il);
        found=true;
      }
    }
    if (!found)
    {
      CNR_ERROR(*m_logger, "unable to find frame " << m_frames.at(idx));
      CNR_ERROR(*m_logger, "Available frames:");
      for (const std::string& s: link_names)
        CNR_ERROR(*m_logger, "- " << s );
    CNR_RETURN_FALSE(*m_logger);
    }
  }

  for (unsigned int idx=0;idx<m_frames.size();idx++)
  {
    add_publisher<geometry_msgs::TwistStamped>( "base_"+m_frames.at(idx), "/"+m_frames.at(idx)+"/twist_in_link",1);
    add_publisher<geometry_msgs::TwistStamped>( "link_"+m_frames.at(idx), "/"+m_frames.at(idx)+"/twist_in_tool",1);
  }
  return true;
}

bool RobotStateController::doUpdate ( const ros::Time& time, const ros::Duration& period )
{
  size_t ll = __LINE__;
  try
  {
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> T_base_links=m_kin->getChain()->getTransformations(q());
    std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> twists=m_kin->getChain()->getTwist(q(),qd());

    for (unsigned int idx=0;idx<m_frames.size();idx++)
    {
      ll = __LINE__;
      Eigen::Vector6d& twist_of_link_in_base=twists.at(m_frame_idxs.at(idx));

      ll = __LINE__;
      Eigen::Vector6d twist_of_link_in_link=rosdyn::spatialRotation(twist_of_link_in_base,T_base_links.at(idx).linear().inverse());

      ll = __LINE__;
      geometry_msgs::TwistStampedPtr msg=boost::make_shared<geometry_msgs::TwistStamped>();
      msg->twist.linear.x=twist_of_link_in_base(0);
      msg->twist.linear.y=twist_of_link_in_base(1);
      msg->twist.linear.z=twist_of_link_in_base(2);

      ll = __LINE__;
      msg->twist.angular.x=twist_of_link_in_base(3);
      msg->twist.angular.y=twist_of_link_in_base(4);
      msg->twist.angular.z=twist_of_link_in_base(5);

      ll = __LINE__;
      msg->header.stamp=ros::Time::now();
      msg->header.frame_id=m_kin->baseLink();
      ll = __LINE__;
      publish("base_"+m_frames.at(idx), msg);

      ll = __LINE__;
      geometry_msgs::TwistStampedPtr msg_in_link=boost::make_shared<geometry_msgs::TwistStamped>();
      msg_in_link->twist.linear.x=twist_of_link_in_link(0);
      msg_in_link->twist.linear.y=twist_of_link_in_link(1);
      msg_in_link->twist.linear.z=twist_of_link_in_link(2);

      ll = __LINE__;
      msg_in_link->twist.angular.x=twist_of_link_in_link(3);
      msg_in_link->twist.angular.y=twist_of_link_in_link(4);
      msg_in_link->twist.angular.z=twist_of_link_in_link(5);

      ll = __LINE__;
      msg_in_link->header.stamp=ros::Time::now();
      msg_in_link->header.frame_id=m_frames.at(idx);

      ll = __LINE__;
      publish("link_"+m_frames.at(idx), msg);
    }
  }
  catch(...)
  {
    CNR_WARN(*m_logger, "Exception in the update method at line: " + std::to_string((long int)(ll)) );
  }
  CNR_RETURN_TRUE(*m_logger);
}





}
