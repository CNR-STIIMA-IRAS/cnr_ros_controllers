#ifndef simple_touch_controller_20190325
#define simple_touch_controller_20190325

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <subscription_notifier/subscription_notifier.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <rosdyn_core/spacevect_algebra.h>
#include <rosdyn_core/primitives.h>
#include <ros/callback_queue.h>
#include <actionlib/server/action_server.h>
#include <thread>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <simple_touch_controller_msgs/simpleTouchAction.h>
#include <tf/transform_listener.h>

#include <cnr_hardware_interface/force_torque_state_interface.h>
#include <cnr_hardware_interface/force_torque_command_interface.h>

#if ROS_VERSION_MINIMUM(1, 14, 1)
#include <memory>
#else
#endif


namespace itia
{
namespace control
{
class SimpleTouchController: public controller_interface::Controller<hardware_interface::ForceTorqueInterface>
{

public:

  bool init     ( hardware_interface::ForceTorqueInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh );
  void update   ( const ros::Time& time, const ros::Duration& period );
  void starting ( const ros::Time& time );
  void stopping ( const ros::Time& time );

protected:
    hardware_interface::ForceTorqueInterface*           m_hw;
    hardware_interface::ForceTorqueHandle               m_ft_h;
    ros::NodeHandle                                     m_root_nh;
    ros::NodeHandle                                     m_controller_nh;
    ros::CallbackQueue                                  m_controller_nh_callback_queue;
    std::string                                         m_ft_resource_name;

    std::shared_ptr< tf::TransformListener > m_listener;        
    ros::Publisher m_target_twist_pub;
    
    std::shared_ptr<actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>>             m_as;
    std::shared_ptr<actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>::GoalHandle> m_gh;
    bool            m_preempted;

    bool            m_touched    = false;
    std::string     m_base_frame;
    std::string     m_tool_frame;
    std::string     m_sensor_frame;
    std::string     m_goal_wrench_frame = "tool";
    std::string     m_goal_twist_frame  = "tool";

    double          m_goal_wrench_norm     ;
    double          m_goal_wrench_norm_toll;
    Eigen::Vector6d m_wrench_s             ;
    Eigen::Vector6d m_target_twist         ;
    Eigen::Vector6d m_goal_wrench_g        ;
    Eigen::Vector6d m_goal_wrench_toll     ;
    Eigen::Vector6d m_goal_wrench_deadband ;
    Eigen::Vector6d m_goal_twist           ;


    std::thread m_as_thread;
    bool m_stop_thread = false;
    void actionGoalCallback   (actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>::GoalHandle gh);
    void actionCancelCallback (actionlib::ActionServer<simple_touch_controller_msgs::simpleTouchAction>::GoalHandle gh);
    void actionThreadTunction ( );

};
}
}







#endif
