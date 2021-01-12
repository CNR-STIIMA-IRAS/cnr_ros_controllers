#include <cnr_joint_state_publisher/cnr_joint_state_publisher.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointStatePublisher,  controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::JointStatePublisher1, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::JointStatePublisher3, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::JointStatePublisher6, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::JointStatePublisher7, controller_interface::ControllerBase)


namespace  cnr
{
namespace control
{

}
}


