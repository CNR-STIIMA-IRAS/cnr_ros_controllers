#ifndef cnr_joint_teleop_controller__20188101642
#define cnr_joint_teleop_controller__20188101642

#include <eigen3/Eigen/Core>

#include <ros/ros.h>

#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>

#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <eigen_state_space_systems/eigen_controllers.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <name_sorting/name_sorting.h>
// #include <joint_teleop_gui/joint_teleop_gui.h>


namespace cnr
{
namespace control
{



/**
 * @brief The JointTeleopController class
 */

class JointTeleopController: public cnr_controller_interface::JointCommandController<hardware_interface::PosVelEffJointInterface>
{

public:
  JointTeleopController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

  void callback(const sensor_msgs::JointStateConstPtr msg);

protected:

};
}
}







#endif
