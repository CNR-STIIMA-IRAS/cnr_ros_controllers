#ifndef cnr_joint_velocity_controller__20188101642
#define cnr_joint_velocity_controller__20188101642

#include <cmath>
#include <Eigen/Core>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

#include <state_space_filters/filtered_values.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>


namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The JointVelocityController class
 */
class JointVelocityController:
    public cnr::control::JointCommandController<
                  hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{
public:
  JointVelocityController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

protected:
  void setPointCallback(const sensor_msgs::JointStateConstPtr &msg);

  std::mutex m_mtx;
  bool       m_has_pos_sp;

  ect::FilteredVectorXd m_vel_fitler_sp;
  rosdyn::VectorXd m_vel_sp;
  rosdyn::VectorXd m_pos_sp;
  rosdyn::VectorXd m_dpos_sp;
  rosdyn::VectorXd m_vel_sp_last;
  rosdyn::VectorXd m_dist_to_pos_sp;
};


}
}

#endif
