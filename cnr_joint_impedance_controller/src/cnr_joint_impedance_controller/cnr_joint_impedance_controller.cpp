#include <name_sorting/name_sorting.h>
#include <cnr_joint_impedance_controller/cnr_joint_impedance_controller.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_inputs.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_outputs.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_state.h>
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
    CNR_ERROR(m_logger, "[ " << getControllerNamespace() << "] Parameter '"<<  P <<"' does not exist");\
    CNR_ERROR(m_logger, "[ " << getControllerNamespace() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }\
  if( X.size() != N )\
  {\
    CNR_ERROR(m_logger, "[ " << getControllerNamespace() << "] The size '"<< X.size() <<"' of the param '" << P << "' does not match with the foreseen dimension '"<< N <<"'");\
    CNR_ERROR(m_logger, "[ " << getControllerNamespace() << "] ERROR DURING INITIALIZATION. ABORT.");\
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
      
      GET_AND_RETURN( getControllerNh(), "sensor_frame"           , m_sensor_link          );
      GET_AND_RETURN( getControllerNh(), "external_wrench_topic"  , external_wrench_topic   );

      m_chain_bs = m_kin->getChain(m_kin->baseLink(),m_sensor_link);
      
      add_subscriber<geometry_msgs::WrenchStamped>(external_wrench_topic,1,
              boost::bind(&cnr::control::JointImpedanceController::setWrenchCallback,this, _1));

      CNR_INFO(m_logger, "[ " << getControllerNamespace() << " ] DOF Chain from Baset to Tool  : " << m_kin->getChain()->getActiveJointsNumber() );
      CNR_INFO(m_logger, "[ " << getControllerNamespace() << " ] DOF Chain from Baset to Sensor: " << m_chain_bs->getActiveJointsNumber() );
    }
    else
    {
      std::string external_torques  = "external_torques";
      GET_AND_RETURN( getControllerNh(), "external_torques_topic", external_torques );
      
      add_subscriber<sensor_msgs::JointState>(external_torques,1,
              boost::bind(&cnr::control::JointImpedanceController::setEffortCallback,this,_1));
    }


    m_q_target.resize(nAx());
    m_qd_target.resize(nAx());
    
    cnr_impedance_regulator::ImpedanceRegulatorOptionsPtr opts(
                                        new cnr_impedance_regulator::ImpedanceRegulatorOptions(nAx()));
    opts->logger = m_logger;
    opts->period = ros::Duration(m_sampling_period);
    opts->robot_kin = m_kin;
    if(!m_regulator.initialize(getRootNh(),getControllerNh(),opts))
    {
      CNR_RETURN_FALSE(m_logger, "Error in initialization of the impedance regulator");
    }

    m_effort_db.resize(nAx());
    m_effort.resize(nAx());
    
    m_q_target.setZero();
    m_qd_target.setZero();
    m_effort.setZero();
    

    std::vector<double> torque_deadband(nAx(),0), wrench_deadband(6,0);
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
      if (torque_deadband.at(iAx)<=0)
      {
        ROS_INFO("torque_deadband value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        m_effort_db (iAx)=0.0;
      }
      else
        m_effort_db (iAx)=torque_deadband.at(iAx);
    }

    m_wrench_db = Eigen::Matrix<double,6,1>( wrench_deadband.data() );
    
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

  cnr_impedance_regulator::ImpedanceRegulatorStatePtr st0(new cnr_impedance_regulator::ImpedanceRegulatorState(m_kin));
  st0->setRobotState(*m_state);
  st0->setModelState(*m_state);
  
  m_regulator.starting(st0, time);

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
    cnr_impedance_regulator::ImpedanceRegulatorInputPtr input;
    cnr_impedance_regulator::ImpedanceRegulatorOutputPtr output;
    input->set_x( m_q_target );
    input->set_xd( m_qd_target );
    input->set_effort( m_effort );
    
    m_regulator.update(nullptr, input, output );

    setCommandPosition( output->get_x() );
    setCommandVelocity( output->get_xd() );
  }
  else
  {
    setCommandPosition(m_regulator.getState0()->getRobotState()->q());
    setCommandVelocity(m_regulator.getState0()->getRobotState()->qd()*0.0);
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
      CNR_WARN(m_logger, "Target Callback - Error in the joint names");
      m_target_ok = false;
      return;
    }
    m_target_ok = true;
    for (unsigned int iAx=0;iAx<nAx();iAx++)
    {
      m_q_target (iAx)=tmp_msg.position.at(iAx);
      m_qd_target(iAx)=tmp_msg.velocity.at(iAx);
    }
  }
  catch(...)
  {
    CNR_WARN(m_logger, "something wrong in Target Callback");
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
      m_effort(iAx) = (tmp_msg.effort.at(iAx) >  m_effort_db (iAx))  ?  tmp_msg.effort.at(iAx) - m_effort_db (iAx)
                    : (tmp_msg.effort.at(iAx) < -m_effort_db (iAx))  ?  tmp_msg.effort.at(iAx) + m_effort_db (iAx)
                    : 0.0;
    }
  }
  catch(...)
  {
    CNR_WARN(m_logger, "something wrong in Effort Callback");
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
      CNR_INFO(m_logger, "sensor frame is "<< msg->header.frame_id <<" it should be " << m_sensor_link);
      return;
    }


    Eigen::Vector6d wrench_of_sensor_in_sensor;
    wrench_of_sensor_in_sensor(0) = (msg->wrench.force.x > m_wrench_db(0)) ? msg->wrench.force.x -m_wrench_db(0)
                                  : (msg->wrench.force.x <-m_wrench_db(0)) ? msg->wrench.force.x +m_wrench_db(0)
                                  : 0.0;                  
    wrench_of_sensor_in_sensor(1) = (msg->wrench.force.y > m_wrench_db(1)) ? msg->wrench.force.y -m_wrench_db(1) 
                                  : (msg->wrench.force.y <-m_wrench_db(1)) ? msg->wrench.force.y +m_wrench_db(1) 
                                  : 0.0;                  
    wrench_of_sensor_in_sensor(2) = (msg->wrench.force.z > m_wrench_db(2)) ? msg->wrench.force.z -m_wrench_db(2) 
                                  : (msg->wrench.force.z <-m_wrench_db(2)) ? msg->wrench.force.z +m_wrench_db(2) 
                                  : 0.0;
    wrench_of_sensor_in_sensor(3) = (msg->wrench.torque.x> m_wrench_db(3)) ? msg->wrench.torque.x-m_wrench_db(3)
                                  : (msg->wrench.torque.x<-m_wrench_db(3)) ? msg->wrench.torque.x+m_wrench_db(3)
                                  : 0.0;
    wrench_of_sensor_in_sensor(4) = (msg->wrench.torque.y> m_wrench_db(4)) ? msg->wrench.torque.y-m_wrench_db(4) 
                                  : (msg->wrench.torque.y<-m_wrench_db(4)) ? msg->wrench.torque.y+m_wrench_db(4) 
                                  : 0.0;
    wrench_of_sensor_in_sensor(5) = (msg->wrench.torque.z> m_wrench_db(5)) ? msg->wrench.torque.z-m_wrench_db(5) 
                                  : (msg->wrench.torque.z<-m_wrench_db(5)) ? msg->wrench.torque.z+m_wrench_db(5) 
                                  : 0.0;

    Eigen::Affine3d T_base_tool              = m_kin->getChain()->getTransformation(this->q());
    Eigen::MatrixXd jacobian_of_tool_in_base = m_kin->getChain()->getJacobian(this->q());
    Eigen::Affine3d T_base_sensor            = m_chain_bs->getTransformation(this->q());
    Eigen::Affine3d T_tool_sensor            = T_base_tool.inverse() * T_base_sensor;
    Eigen::Vector6d wrench_of_tool_in_tool   = rosdyn::spatialDualTranformation(wrench_of_sensor_in_sensor,T_tool_sensor);
    Eigen::Vector6d wrench_of_tool_in_base   = rosdyn::spatialRotation(wrench_of_tool_in_tool,T_base_tool.linear());
    m_effort                                 = jacobian_of_tool_in_base.transpose()*wrench_of_tool_in_base;
    m_effort_ok = true;
  }
  catch(...)
  {
    CNR_WARN(m_logger, "something wrong in wrench callback");
    m_effort_ok=false;
  }
}




}
}
