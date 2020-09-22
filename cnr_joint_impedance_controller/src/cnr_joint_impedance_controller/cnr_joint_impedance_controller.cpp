#include <cnr_joint_impedance_controller/cnr_joint_impedance_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointImpedanceController, controller_interface::ControllerBase)

namespace std
{
inline std::string to_string( const std::vector<std::string>& vals )
{
  std::string ret = "< ";
  for( auto const & val : vals ) ret += val + ", ";
  ret += " >";
  return ret;
}

inline std::string to_string( const std::string& val )
{
  return val;
}

inline std::string to_string( const bool& val )
{
  return val ? "TRUE" : "FALSE";
}

}


#define GET_AND_RETURN( nh, param, value )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_ERROR("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    return false;\
  }



#define GET_AND_DEFAULT( nh, param, value, def )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_WARN("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    ROS_WARN("Default value '%s' superimposed. ", std::to_string( def ).c_str() );\
    value=def;\
  }


namespace cnr
{
namespace control
{

JointImpedanceController::~JointImpedanceController()
{

}

bool JointImpedanceController::doInit( )
#define GET_PARAM_VECTOR_AND_RETURN( P, X , N)\
  if (!getControllerNh().getParam( std::string(P).c_str(), X))\
  {\
    ROS_FATAL_STREAM("[ " << getControllerNamespace() << "] Parameter '"<<  P <<"' does not exist");\
    ROS_FATAL_STREAM("[ " << getControllerNamespace() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }\
  if( X.size() != N )\
  {\
    ROS_FATAL_STREAM("[ " << getControllerNamespace() << "] The size '"<< X.size() <<"' of the param '" << P << "' does not match with the foreseen dimension '"<< N <<"'");\
    ROS_FATAL_STREAM("[ " << getControllerNamespace() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }
{
  CNR_TRACE_START(*m_logger);

  try
  { 
    m_is_configured = false;
    m_target_ok     = false;
    m_effort_ok     = false;

    std::string joint_target      = "joint_target";
    
    GET_AND_RETURN( getControllerNh(), "setpoint", joint_target);

    CNR_INFO(*m_logger, "Subscribing " << joint_target);
    add_subscriber<sensor_msgs::JointState>(joint_target, 1,
            boost::bind(&cnr::control::JointImpedanceController::setTargetCallback,this,_1));

    if (!getControllerNh().getParam("use_wrench", m_use_wrench))
    {
      ROS_WARN_STREAM(getControllerNh().getNamespace()+"/'use_wrench' does not exist. Default value false.");
      m_use_wrench = false;
    }

    if (m_use_wrench)
    {
      std::string external_wrench_topic;
      std::string robot_description_param;
      urdf::Model urdf_model;
      
      GET_AND_RETURN( getControllerNh(), "robot_description_param", robot_description_param );
      GET_AND_RETURN( getControllerNh(), "sensor_frame"           , m_sensor_link          );
      GET_AND_RETURN( getControllerNh(), "external_wrench_topic"  , external_wrench_topic   );

      urdf_model.initParam( robot_description_param.c_str() );

      Eigen::Vector3d gravity;
      gravity << 0, 0, -9.806;
      m_chain_bs = rosdyn::createChain(urdf_model,m_kin->baseLink(),m_sensor_link,gravity);
      
      add_subscriber<geometry_msgs::WrenchStamped>(external_wrench_topic,1,
              boost::bind(&cnr::control::JointImpedanceController::setWrenchCallback,this, _1));

      ROS_INFO_STREAM("[ " << getControllerNamespace() << " ] DOF Chain from Baset to Tool  : " << m_kin->getChain()->getActiveJointsNumber() );
      ROS_INFO_STREAM("[ " << getControllerNamespace() << " ] DOF Chain from Baset to Sensor: " << m_chain_bs->getActiveJointsNumber() );

    }
    else
    {
      std::string external_torques  = "external_torques";
      GET_AND_RETURN( getControllerNh(), "external_torques_topic", external_torques );
      
      add_subscriber<sensor_msgs::JointState>(external_torques,1,
              boost::bind(&cnr::control::JointImpedanceController::setEffortCallback,this,_1));
    }


    m_jtarget.resize(nAx());
    m_jDtarget.resize(nAx());
    m_x0.resize(nAx());
    m_Dx0.resize(nAx());
    m_x.resize(nAx());
    m_Dx.resize(nAx());
    m_DDx.resize(nAx());
    
    m_Jinv.resize(nAx());
    m_damping.resize(nAx());
    m_damping_dafault.resize(nAx());
    m_k.resize(nAx());
    m_k_default.resize(nAx());
    m_k_new.resize(nAx());
    m_torque_deadband.resize(nAx());
    m_torque.resize(nAx());
    
    m_jtarget.setZero();
    m_jDtarget.setZero();
    m_x0.setZero();
    m_Dx0.setZero();
    m_x.setZero();
    m_Dx.setZero();
    m_DDx.setZero();
    m_torque.setZero();
    

    std::vector<double> inertia(nAx(),0), damping(nAx(),0), stiffness(nAx(),0), torque_deadband(nAx(),0), wrench_deadband(6,0);
    GET_PARAM_VECTOR_AND_RETURN( "inertia"  , inertia  , nAx());
    GET_PARAM_VECTOR_AND_RETURN( "stiffness", stiffness, nAx());

    if (getControllerNh().hasParam("damping_ratio"))
    {
      std::vector<double> damping_ratio;
      GET_PARAM_VECTOR_AND_RETURN( "damping_ratio", damping_ratio, nAx());

      damping.resize(nAx(),0);
      for (unsigned int iAx=0; iAx<nAx(); iAx++)
      {
        if (stiffness.at(iAx)<=0)
        {
          ROS_ERROR("damping ratio can be specified only for positive stiffness values (stiffness of Joint is not positive)");
          return false;
        }
        damping.at(iAx)=2*damping_ratio.at(iAx)*std::sqrt(stiffness.at(iAx)*inertia.at(iAx));
      }
    }
    else
    {
      GET_PARAM_VECTOR_AND_RETURN("damping", damping, nAx());
    }


    if (m_use_wrench)
    {
      GET_PARAM_VECTOR_AND_RETURN("wrench_deadband", wrench_deadband, 6);
    }
    else
    {
      GET_PARAM_VECTOR_AND_RETURN("torque_deadband", torque_deadband, nAx());
    }

    for (unsigned int iAx=0;iAx<nAx();iAx++)
    {
      if (inertia.at(iAx)<=0)
      {
        ROS_INFO("inertia value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        m_Jinv(iAx)=0.0;
      }
      else
        m_Jinv(iAx)=1.0/inertia.at(iAx);
      
      if (damping.at(iAx)<=0)
      {
        ROS_INFO("damping value of Joint %d is not positive, setting equalt to 10/inertia",iAx);
        m_damping(iAx)         = 10.0 * m_Jinv(iAx);
        m_damping_dafault(iAx) = 10.0 * m_Jinv(iAx);
      }
      else
      {
        m_damping(iAx)          = damping.at(iAx);
        m_damping_dafault(iAx)  = damping.at(iAx);
      }
      
      
      if (stiffness.at(iAx)<0)
      {
        ROS_INFO("maximum fitness value of Joint %d is negative, setting equal to 0",iAx);
        m_k(iAx)=0.0;
        m_k_default(iAx)=0.0;
      }
      else
      {
        m_k(iAx)=stiffness.at(iAx);
        m_k_default(iAx)=stiffness.at(iAx);
      }

      if (torque_deadband.at(iAx)<=0)
      {
        ROS_INFO("torque_deadband value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        m_torque_deadband(iAx)=0.0;
      }
      else
        m_torque_deadband(iAx)=torque_deadband.at(iAx);
    }

    m_wrench_deadband = Eigen::Matrix<double,6,1>( wrench_deadband.data() );
    
  }
  catch(const  std::exception& e)
  {
    ROS_FATAL("EXCEPTION: %s", e.what());
    return false;
  }
  ROS_INFO("[ %s ] init OK controller",  getControllerNh().getNamespace().c_str());

  this->setPriority(QD_PRIORITY);

  return true;
#undef GET_PARAM_VECTOR_AND_RETURN
}

bool JointImpedanceController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(*m_logger);
  m_x = q();
  m_Dx = qd();

  m_x0      = m_x;
  m_Dx0     = m_Dx;
  m_jtarget  = m_x;
  m_jDtarget = m_Dx;

  CNR_RETURN_TRUE(*m_logger);
}

bool JointImpedanceController::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  // ==================================================================
  bool is_configured = m_is_configured;
  m_is_configured    = m_target_ok && m_effort_ok;
  if (m_is_configured && !is_configured)
  {
    ROS_INFO("Joint Impedance Controller Configured");
  }
  // ==================================================================

  if (m_is_configured)
  {
    m_DDx = m_Jinv.cwiseProduct( m_k.cwiseProduct(m_jtarget-m_x) + m_damping.cwiseProduct(m_jDtarget - m_Dx) + m_torque );
    m_x  += m_Dx  * period.toSec() + m_DDx*std::pow(period.toSec(),2.0)*0.5;
    m_Dx += m_DDx * period.toSec();
    ROS_DEBUG_STREAM_THROTTLE(2, "x      : " << m_x.transpose() );
    ROS_DEBUG_STREAM_THROTTLE(2, "Dx     : " << m_Dx.transpose() );
    ROS_DEBUG_STREAM_THROTTLE(2, "Torque : " << m_torque.transpose() );

    setCommandPosition( m_x );
    setCommandVelocity( m_Dx );
  }
  else
  {
    setCommandPosition( m_x0 );
    setCommandVelocity( m_Dx * 0.0 );
  }
  CNR_RETURN_TRUE(*m_logger);
}

void JointImpedanceController::setTargetCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg)
{
  try 
  {
    sensor_msgs::JointState tmp_msg = *msg;
    if (!name_sorting::permutationName(jointNames(),tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort, "JOINT IMP CTRL - SET TARGET CALLBACK"))
    {
      ROS_WARN("[ %s ] Target Callback - Error in the joint names",  getControllerNh().getNamespace().c_str());
      m_target_ok = false;
      return;
    }
    ROS_DEBUG_ONCE( "JOINT TARGET TARGET RECEIVED!");
    m_target_ok = true;
    for (unsigned int iAx=0;iAx<nAx();iAx++)
    {
      m_jtarget (iAx)=tmp_msg.position.at(iAx);
      m_jDtarget(iAx)=tmp_msg.velocity.at(iAx);
    }
  }
  catch(...)
  {
    ROS_WARN("[ %s ] something wrong in Target Callback",  getControllerNh().getNamespace().c_str());
    m_target_ok=false;
  }
}
void JointImpedanceController::setEffortCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg)
{

  try
  {

    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(jointNames(),tmp_msg.name,tmp_msg.effort, "JOINT IMP CTRL - set EFFORT CALLBACK"))
    {
      ROS_ERROR("joints not found");
      m_effort_ok=false;
      return;
    }
    ROS_DEBUG_ONCE( "EFFORT FEEDBACK RECEIVED!");
    m_effort_ok=true;
    for (unsigned int iAx=0;iAx<nAx();iAx++)
    {
      m_torque(iAx) = ( tmp_msg.effort.at(iAx) >  m_torque_deadband(iAx) )  ?  tmp_msg.effort.at(iAx) - m_torque_deadband(iAx)
                    : ( tmp_msg.effort.at(iAx) < -m_torque_deadband(iAx) )  ?  tmp_msg.effort.at(iAx) + m_torque_deadband(iAx)
                    : 0.0;
    }
  }
  catch(...)
  {
    ROS_WARN("[ %s ] something wrong in Effort Callback",  getControllerNh().getNamespace().c_str());
    m_effort_ok=false;
  }
}

void JointImpedanceController::setWrenchCallback(const boost::shared_ptr<geometry_msgs::WrenchStamped const>& msg)
{
  size_t ll = 0;
  try
  {

    ll = __LINE__;
    if (msg->header.frame_id.compare(m_sensor_link))
    {
      ROS_INFO("[ %s ] sensor frame is %s, it should be %s", getControllerNamespace().c_str(),msg->header.frame_id.c_str(),m_sensor_link.c_str());
      return;
    }


    Eigen::Vector6d wrench_of_sensor_in_sensor;
    ll = __LINE__;
    wrench_of_sensor_in_sensor(0) = ( msg->wrench.force.x  >  m_wrench_deadband(0) )  ?  msg->wrench.force.x  - m_wrench_deadband(0)
                                  : ( msg->wrench.force.x  < -m_wrench_deadband(0) )  ?  msg->wrench.force.x  + m_wrench_deadband(0)
                                  : 0.0;
    wrench_of_sensor_in_sensor(1) = ( msg->wrench.force.y  >  m_wrench_deadband(1) )  ?  msg->wrench.force.y  - m_wrench_deadband(1) : ( msg->wrench.force.y  < -m_wrench_deadband(1) )  ?  msg->wrench.force.y  + m_wrench_deadband(1) : 0.0;
    wrench_of_sensor_in_sensor(2) = ( msg->wrench.force.z  >  m_wrench_deadband(2) )  ?  msg->wrench.force.z  - m_wrench_deadband(2) : ( msg->wrench.force.z  < -m_wrench_deadband(2) )  ?  msg->wrench.force.z  + m_wrench_deadband(2) : 0.0;
    wrench_of_sensor_in_sensor(3) = ( msg->wrench.torque.x >  m_wrench_deadband(3) )  ?  msg->wrench.torque.x - m_wrench_deadband(3) : ( msg->wrench.torque.x < -m_wrench_deadband(3) )  ?  msg->wrench.torque.x + m_wrench_deadband(3) : 0.0;
    wrench_of_sensor_in_sensor(4) = ( msg->wrench.torque.y >  m_wrench_deadband(4) )  ?  msg->wrench.torque.y - m_wrench_deadband(4) : ( msg->wrench.torque.y < -m_wrench_deadband(4) )  ?  msg->wrench.torque.y + m_wrench_deadband(4) : 0.0;
    wrench_of_sensor_in_sensor(5) = ( msg->wrench.torque.z >  m_wrench_deadband(5) )  ?  msg->wrench.torque.z - m_wrench_deadband(5) : ( msg->wrench.torque.z < -m_wrench_deadband(5) )  ?  msg->wrench.torque.z + m_wrench_deadband(5) : 0.0;

    ll = __LINE__; Eigen::Affine3d T_base_tool              = m_kin->getChain()->getTransformation(m_x);
    ll = __LINE__; Eigen::MatrixXd jacobian_of_tool_in_base = m_kin->getChain()->getJacobian(m_x);
    ll = __LINE__; Eigen::Affine3d T_base_sensor            = m_chain_bs->getTransformation(m_x);
    ll = __LINE__; Eigen::Affine3d T_tool_sensor            = T_base_tool.inverse()*T_base_sensor;
    ll = __LINE__; Eigen::Vector6d wrench_of_tool_in_tool   = rosdyn::spatialDualTranformation(wrench_of_sensor_in_sensor,T_tool_sensor);
    ll = __LINE__; Eigen::Vector6d wrench_of_tool_in_base   = rosdyn::spatialRotation(wrench_of_tool_in_tool,T_base_tool.linear());
    ll = __LINE__; m_torque                                 = jacobian_of_tool_in_base.transpose()*wrench_of_tool_in_base;
    ll = __LINE__; m_effort_ok = true;
  }
  catch(...)
  {
    ROS_WARN("[ %s ] something wrong in wrench callback",  getControllerNh().getNamespace().c_str());
    m_effort_ok=false;
  }
}




}
}
