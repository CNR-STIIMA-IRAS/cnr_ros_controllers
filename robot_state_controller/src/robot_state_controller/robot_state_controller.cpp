#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <robot_state_controller_msgs/PoseTwistArray.h>
#include <robot_state_controller_msgs/PoseTwist.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_core/urdf_parser.h>
#include <robot_state_controller/robot_state_controller.h>
#include <pluginlib/class_list_macros.h> // header for PLUGINLIB_EXPORT_CLASS. NOTE IT SHOULD STAY IN THE CPP FILE NOTE

PLUGINLIB_EXPORT_CLASS(cnr::control::RobotStateController  , controller_interface::ControllerBase)

namespace cnr
{
namespace control
{

//!
bool RobotStateController::doInit( )
{

  if (!this->getControllerNh().getParam("frames",m_frames))
  {
    CNR_RETURN_FALSE(this->logger(), "Param 'frames' not defined");
  }

  if (!this->getControllerNh().getParam("publish_twist_as_separate",m_publish_twist_as_separate))
  {
    m_publish_twist_as_separate=false;
  }

  std::vector<std::string> link_names=this->chain().getLinksName();

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
      CNR_ERROR(this->logger(), "unable to find frame " << m_frames.at(idx));
      CNR_ERROR(this->logger(), "Available frames:");
      for (const std::string& s: link_names)
        CNR_ERROR(this->logger(), "- " << s );
    CNR_RETURN_FALSE(this->logger());
    }
  }

  if (m_publish_twist_as_separate)
  {
    for (unsigned int idx=0;idx<m_frames.size();idx++)
    {
      size_t idx_base = this->template add_publisher<geometry_msgs::TwistStamped>("/"+m_frames.at(idx)+"/twist_in_base",1);
      m_base_pub_idx.push_back(idx_base);
      size_t idx_link = this-> template add_publisher<geometry_msgs::TwistStamped>("/"+m_frames.at(idx)+"/twist_in_tool",1);
      m_link_pub_idx.push_back(idx_link);
    }
  }
  else
  {
    size_t idx_base = this->template add_publisher<robot_state_controller_msgs::PoseTwistArray>("pose_twist",1);
    m_base_pub_idx.push_back(idx_base);
    m_link_pub_idx.clear();
  }

  return true;
}

//!
bool RobotStateController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  size_t ll = __LINE__;
  try
  {
    static Eigen::VectorXd _q(this->nAx());
    static Eigen::VectorXd _qd(this->nAx());
    _q = Eigen::Map<const Eigen::VectorXd>(this->chainState().handle_to_q(), this->nAx());
    _qd = Eigen::Map<const Eigen::VectorXd>(this->chainState().handle_to_qd(), this->nAx());
    auto T_base_links = this->chainState().linkPose();
    auto twists       = this->chainState().linkTwist();

    if (m_publish_twist_as_separate)
    {
      for (unsigned int idx=0;idx<m_frames.size();idx++)
      {
        ll = __LINE__;
        Eigen::Vector6d twist_of_link_in_base=twists.at(m_frame_idxs.at(idx));

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
        msg->header.frame_id=this->chain().getLinksName().front();

        ll = __LINE__;
        if(!this->publish(m_base_pub_idx.at(idx), msg))
        {
          CNR_RETURN_FALSE(this->logger());
        }

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
        if(!this->publish(m_link_pub_idx.at(idx), msg))
        {
          CNR_RETURN_FALSE(this->logger());
        }
      }
    }
    else
    {
      robot_state_controller_msgs::PoseTwistArrayPtr msg=boost::make_shared<robot_state_controller_msgs::PoseTwistArray>();
      for (unsigned int idx=0;idx<m_frames.size();idx++)
      {
        ll = __LINE__;
        Eigen::Vector3d position_of_link_in_base=T_base_links.at(m_frame_idxs.at(idx)).translation();
        Eigen::Quaterniond orientation_of_link_in_base(T_base_links.at(m_frame_idxs.at(idx)).linear());

        ll = __LINE__;
        Eigen::Vector6d twist_of_link_in_base=twists.at(m_frame_idxs.at(idx));

        ll = __LINE__;
        robot_state_controller_msgs::PoseTwist msg_el;
        msg_el.pose.position.x=position_of_link_in_base(0);
        msg_el.pose.position.y=position_of_link_in_base(1);
        msg_el.pose.position.z=position_of_link_in_base(2);

        ll = __LINE__;
        msg_el.pose.orientation.x=orientation_of_link_in_base.coeffs()(0);
        msg_el.pose.orientation.y=orientation_of_link_in_base.coeffs()(1);
        msg_el.pose.orientation.z=orientation_of_link_in_base.coeffs()(2);
        msg_el.pose.orientation.w=orientation_of_link_in_base.coeffs()(3);

        ll = __LINE__;
        msg_el.twist.linear.x=twist_of_link_in_base(0);
        msg_el.twist.linear.y=twist_of_link_in_base(1);
        msg_el.twist.linear.z=twist_of_link_in_base(2);

        ll = __LINE__;
        msg_el.twist.angular.x=twist_of_link_in_base(3);
        msg_el.twist.angular.y=twist_of_link_in_base(4);
        msg_el.twist.angular.z=twist_of_link_in_base(5);

        ll = __LINE__;
        msg->pose_twist_array.push_back(msg_el);
      }

      ll = __LINE__;
      msg->header.stamp=ros::Time::now();
      msg->header.frame_id=this->chain().getLinksName().front();

      ll = __LINE__;
      if(!this->publish(m_base_pub_idx.at(0), msg))
      {
        CNR_RETURN_FALSE(this->logger());
      }
    }


  }
  catch(...)
  {
    CNR_WARN(this->logger(), "Exception in the update method at line: " + std::to_string((long int)(ll)) );
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}



}
}
