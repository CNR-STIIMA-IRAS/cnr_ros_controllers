#pragma once // worksaround qtcreator clang-tody

#ifndef cnr_joint_impedance_controller__cnr_joint_impedance_controller_impl_h
#define cnr_joint_impedance_controller__cnr_joint_impedance_controller_impl_h

#include <rosdyn_core/primitives.h>
#include <name_sorting/name_sorting.h>
#include <cnr_joint_impedance_controller/cnr_joint_impedance_controller.h>
#include <cnr_regulator_interface/cnr_regulator_references.h>
#include <cnr_regulator_interface/cnr_regulator_control_commands.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_state.h>
#include <cnr_impedance_regulator/cnr_impedance_regulator_params.h>
#include <pluginlib/class_list_macros.h>

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

//!
inline JointImpedanceController::~JointImpedanceController()
{

}

//!
inline bool JointImpedanceController::doInit( )
#define GET_PARAM_VECTOR_AND_RETURN( P, X , N)\
  if(!this->getControllerNh().getParam( std::string(P).c_str(), X))\
  {\
    CNR_ERROR(this->logger(), "[ " <<this->getControllerNamespace() << "] Parameter '"<<  P <<"' does not exist");\
    CNR_ERROR(this->logger(), "[ " <<this->getControllerNamespace() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }\
  if( X.size() != N )\
  {\
    CNR_ERROR(this->logger(), "[ " <<this->getControllerNamespace() << "] The size '"<< X.size() <<"' of the param '" << P << "' does not match with the foreseen dimension '"<< N <<"'");\
    CNR_ERROR(this->logger(), "[ " <<this->getControllerNamespace() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }
{
  CNR_TRACE_START(this->logger());

  try
  { 
    m_is_configured = false;
    m_target_ok     = false;
    m_effort_ok     = false;

    std::string joint_target      = "joint_target";
    
    GET_AND_RETURN(this->getControllerNh(), "setpoint", joint_target);

    CNR_INFO(*this->logger(), "Subscribing " << joint_target);
    this->template add_subscriber<sensor_msgs::JointState>(joint_target, 1,
            boost::bind(&cnr::control::JointImpedanceController::setTargetCallback,this,_1));

    if(!this->getControllerNh().getParam("use_wrench", m_use_wrench))
    {
      ROS_WARN_STREAM(this->getControllerNh().getNamespace()+"/'use_wrench' does not exist. Default value false.");
      m_use_wrench = false;
    }

    if (m_use_wrench)
    {
      std::string external_wrench_topic;
      std::string robot_description_param;
      
      GET_AND_RETURN(this->getControllerNh(), "sensor_frame"           , m_sensor_link          );
      GET_AND_RETURN(this->getControllerNh(), "external_wrench_topic"  , external_wrench_topic   );

      m_root_link.fromUrdf(this->m_urdf_model->root_link_.get());

      std::string error;
      if(!m_chain_bs.init(error,&m_root_link,this->m_chain.getLinksName().front(), m_sensor_link))
      {
        CNR_ERROR(this->m_logger, "Failing in creating the Chain from the URDF model:\n\t" + error + "");
        CNR_RETURN_FALSE(this->m_logger);
      }
      
      this->template add_subscriber<geometry_msgs::WrenchStamped>(external_wrench_topic,1,
              boost::bind(&cnr::control::JointImpedanceController::setWrenchCallback,this, _1));

      CNR_INFO(this->logger(), "[ " <<this->getControllerNamespace() << " ] DOF Chain from Baset to Tool  : " << this->m_chain.getActiveJointsNumber() );
      CNR_INFO(this->logger(), "[ " <<this->getControllerNamespace() << " ] DOF Chain from Baset to Sensor: " << m_chain_bs.getActiveJointsNumber() );
    }
    else
    {
      std::string external_torques  = "external_torques";
      GET_AND_RETURN(this->getControllerNh(), "external_torques_topic", external_torques );
      
      this->template add_subscriber<sensor_msgs::JointState>(external_torques,1,
              boost::bind(&cnr::control::JointImpedanceController::setEffortCallback,this,_1));
    }


    eigen_utils::resize(m_q_target, this->nAx());
    eigen_utils::resize(m_qd_target,this->nAx());
    
    ImpedanceRegulatorParamsPtr opts(new ImpedanceRegulatorParams(this->nAx()));
    opts->logger = this->logger();
    opts->period = ros::Duration(this->m_sampling_period);
    opts->resources_names = this->m_chain.getActiveJointsName();
    if(!m_regulator.initialize(this->getRootNh(),this->getControllerNh(),opts))
    {
      CNR_RETURN_FALSE(this->logger(), "Error in initialization of the impedance regulator");
    }

    eigen_utils::resize(m_effort_db,this->nAx());
    eigen_utils::resize(m_effort,this->nAx());
    
    eigen_utils::setZero(m_q_target);
    eigen_utils::setZero(m_qd_target);
    eigen_utils::setZero(m_effort);

    std::vector<double> torque_deadband(this->nAx(),0), wrench_deadband(6,0);
    if (m_use_wrench)
    {
      GET_PARAM_VECTOR_AND_RETURN("wrench_deadband", wrench_deadband, 6);
    }
    else
    {
      GET_PARAM_VECTOR_AND_RETURN("torque_deadband", torque_deadband, this->nAx());
    }

    for (unsigned int iAx=0;iAx<this->nAx();iAx++)
    {
      if (torque_deadband.at(iAx)<=0)
      {
        ROS_INFO("torque_deadband value of Joint %d is not positive, disabling impedance control for this axis",iAx);
        eu::at(m_effort_db,iAx)=0.0;
      }
      else
        eu::at(m_effort_db,iAx)=torque_deadband.at(iAx);
    }

    m_wrench_db = Eigen::Matrix<double,6,1>( wrench_deadband.data() );
    
  }
  catch(const  std::exception& e)
  {
    ROS_FATAL("EXCEPTION: %s", e.what());
    return false;
  }
  ROS_INFO("[ %s ] init OK controller", this->getControllerNh().getNamespace().c_str());

  this->setPriority(this->QD_PRIORITY);

  return true;
#undef GET_PARAM_VECTOR_AND_RETURN
}

//!
inline bool JointImpedanceController::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());

  typename ImpedanceRegulatorState::Ptr st0(new ImpedanceRegulatorState(this->m_chain));
  st0->robotState().copy(this->chainState(), this->chainState().ONLY_JOINT);
  st0->msdState() = this->chainState();
  
  m_regulator.starting(st0, time); 
  m_regulator_input.reset(new JointRegulatorReference());
  m_regulator_input->set_dimension(this->nAx());
  m_regulator_output.reset(new JointRegulatorControlCommand());
  m_regulator_output->set_dimension(this->nAx());

  CNR_RETURN_TRUE(this->logger());
}

//!
inline bool JointImpedanceController::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
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
    m_regulator_input->q = m_q_target;
    m_regulator_input->qd = m_qd_target;
    m_regulator_input->effort = m_effort;
    
    m_regulator.update(m_regulator_input, m_regulator_output);

    this->setCommandPosition( m_regulator_output->x );
    this->setCommandVelocity( m_regulator_output->xd );
  }
  else
  {
    this->setCommandPosition(m_regulator.x0()->robotState().q());
    this->setCommandVelocity(m_regulator.x0()->robotState().qd() * 0.0);
  }
  CNR_RETURN_TRUE(this->logger());
}

//!
inline void JointImpedanceController::setTargetCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg)
{
  try 
  {
    sensor_msgs::JointState tmp_msg = *msg;
    if (!name_sorting::permutationName(this->jointNames(),
                                       tmp_msg.name,tmp_msg.position,tmp_msg.velocity,tmp_msg.effort,
                                       "JOINT IMP CTRL - SET TARGET CALLBACK"))
    {
      CNR_WARN(this->logger(), "Target Callback - Error in the joint names");
      m_target_ok = false;
      return;
    }
    m_target_ok = true;
    for (unsigned int iAx=0;iAx<this->nAx();iAx++)
    {
      eu::at(m_q_target,iAx)=tmp_msg.position.at(iAx);
      eu::at(m_qd_target,iAx)=tmp_msg.velocity.at(iAx);
    }
  }
  catch(...)
  {
    CNR_WARN(this->logger(), "something wrong in Target Callback");
    m_target_ok=false;
  }
}


//!
inline void JointImpedanceController::setEffortCallback(const boost::shared_ptr<sensor_msgs::JointState const>& msg)
{

  try
  {

    sensor_msgs::JointState tmp_msg=*msg;
    if (!name_sorting::permutationName(this->jointNames(),tmp_msg.name,tmp_msg.effort, "JOINT IMP CTRL - set EFFORT CALLBACK"))
    {
      ROS_ERROR("joints not found");
      m_effort_ok=false;
      return;
    }
    ROS_DEBUG_ONCE( "EFFORT FEEDBACK RECEIVED!");
    m_effort_ok=true;
    for (unsigned int iAx=0;iAx<this->nAx();iAx++)
    {
      eu::at(m_effort,iAx)= (tmp_msg.effort.at(iAx) >  eu::at(m_effort_db,iAx))  ?  tmp_msg.effort.at(iAx) - eu::at(m_effort_db,iAx)
                          : (tmp_msg.effort.at(iAx) < -eu::at(m_effort_db,iAx))  ?  tmp_msg.effort.at(iAx) + eu::at(m_effort_db,iAx)
                          : 0.0;
    }
  }
  catch(...)
  {
    CNR_WARN(this->logger(), "something wrong in Effort Callback");
    m_effort_ok=false;
  }
}

//!
inline void JointImpedanceController::setWrenchCallback(const boost::shared_ptr<geometry_msgs::WrenchStamped const>& msg)
{
  try
  {
    if (msg->header.frame_id.compare(m_sensor_link))
    {
      CNR_INFO(this->logger(), "sensor frame is "<< msg->header.frame_id <<" it should be " << m_sensor_link);
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


    Eigen::Affine3d T_base_tool              = this->chainState().toolPose();
    rosdyn::Matrix6Xd jacobian_of_tool_in_base = this->chainState().jacobian();

    rosdyn::VectorXd _q(this->m_chain.getActiveJointsNumber());
    eu::copy(_q,this->getPosition());
    Eigen::Affine3d T_base_sensor            = m_chain_bs.getTransformation(_q);
    Eigen::Affine3d T_tool_sensor            = T_base_tool.inverse() * T_base_sensor;
    Eigen::Vector6d wrench_of_tool_in_tool   = rosdyn::spatialDualTranformation(wrench_of_sensor_in_sensor,T_tool_sensor);
    Eigen::Vector6d wrench_of_tool_in_base   = rosdyn::spatialRotation(wrench_of_tool_in_tool,T_base_tool.linear());
    m_effort                                 = jacobian_of_tool_in_base.transpose() * wrench_of_tool_in_base;
    m_effort_ok = true;
  }
  catch(...)
  {
    CNR_WARN(this->logger(), "something wrong in wrench callback");
    m_effort_ok=false;
  }
}




}
}

#endif
