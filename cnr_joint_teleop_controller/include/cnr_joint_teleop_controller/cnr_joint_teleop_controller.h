#ifndef cnr_joint_teleop_controller__20188101642
#define cnr_joint_teleop_controller__20188101642

#include <cmath>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>

#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <sensor_msgs/JointState.h>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <name_sorting/name_sorting.h>

namespace cnr
{
namespace control
{



/**
 * @brief The JointTeleopController class
 */

class JointTeleopController:
    public cnr_controller_interface::JointCommandController<hardware_interface::PosVelEffJointInterface>
{

public:
  JointTeleopController();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);
  void callback(const sensor_msgs::JointStateConstPtr &msg);

protected:

  std::mutex      m_mtx;
  bool            m_has_pos_sp;
  Eigen::VectorXd m_vel_sp;
  Eigen::VectorXd m_pos_sp;
  Eigen::VectorXd m_dpos_sp;
  Eigen::VectorXd m_vel_sp_last;
  Eigen::VectorXd m_dist_to_pos_sp;
  Eigen::VectorXd m_scaling_factor;

  struct DumpFilter
  {
    bool          active;
    double        dump_time;
    ros::WallTime last_msg_time;
    double dumpFactor()
    {
      if(!active)
        return 0.0;

      double dt = (ros::WallTime::now() - last_msg_time).toSec();
      return (dt > dump_time) ? 0.0 : std::cos( (dt/dump_time) * M_PI/2.0 );
    }
    DumpFilter() = default;
    DumpFilter(double time_window) : active(false), dump_time( time_window ){}
    void tick()
    {
      active |= true;
      last_msg_time = ros::WallTime::now();
    }
  } m_dump;



};
}
}







#endif
