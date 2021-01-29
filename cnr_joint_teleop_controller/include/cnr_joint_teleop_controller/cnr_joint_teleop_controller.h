#ifndef cnr_joint_teleop_controller__20188101642
#define cnr_joint_teleop_controller__20188101642

#include <cmath>
#include <Eigen/Core>
#include <ros/ros.h>

#include <eigen_state_space_systems/controllers/controllers.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <sensor_msgs/JointState.h>

namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The JointTeleopController class
 */

template<int N,int MaxN=N>
class JointTeleopControllerN:
    public cnr::control::JointCommandController<N, MaxN,
               hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
{

public:
  JointTeleopControllerN();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);
  void callback(const sensor_msgs::JointStateConstPtr &msg);

protected:

  std::mutex      m_mtx;
  bool            m_has_pos_sp;
  ect::Value<N,MaxN> m_vel_sp;
  ect::Value<N,MaxN> m_pos_sp;
  ect::Value<N,MaxN> m_dpos_sp;
  ect::Value<N,MaxN> m_vel_sp_last;
  ect::Value<N,MaxN> m_dist_to_pos_sp;
  ect::Value<N,MaxN> m_scaling_factor;

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

using JointTeleopController  = JointTeleopControllerN<-1, cnr::control::max_num_axes>;
using JointTeleopController1 = JointTeleopControllerN<1>;
using JointTeleopController3 = JointTeleopControllerN<3>;
using JointTeleopController6 = JointTeleopControllerN<6>;
using JointTeleopController7 = JointTeleopControllerN<7>;
}
}

#include <cnr_joint_teleop_controller/internal/cnr_joint_teleop_controller_impl.h>



#endif
