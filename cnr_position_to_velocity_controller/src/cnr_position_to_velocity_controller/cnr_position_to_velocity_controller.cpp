#include <cnr_position_to_velocity_controller/cnr_position_to_velocity_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::PositionToVelocityController  , controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(cnr::control::PositionToVelocityControllerFfw  , controller_interface::ControllerBase)

namespace ect = eigen_control_toolbox;
namespace eu  = eigen_utils;

namespace cnr
{
namespace control
{

template<class H, class T>
bool PositionToVelocityControllerBase<H,T>::doInit()
{
  CNR_TRACE_START(this->logger());
//  if(this->nAx()>1)
//  {
//    CNR_ERROR(this->logger(), "The controller is designed to control only one joint.");
//    CNR_RETURN_FALSE(this->logger());
//  }

  std::string setpoint_topic_name;
  //std::string feedforward_topic_name;

  if(!this->getControllerNh().getParam("setpoint_topic_name", setpoint_topic_name))
  {
    CNR_ERROR(this->logger(), this->getControllerNamespace() + "/'setpoint_topic_name' does not exist");
    CNR_ERROR(this->logger(), "ERROR DURING INITIALIZATION CONTROLLER "<< this->getControllerNamespace());
    CNR_RETURN_FALSE(this->logger());
  }

  this->template add_subscriber<sensor_msgs::JointState>(setpoint_topic_name, 1,
              boost::bind(&PositionToVelocityControllerBase<H,T>::callback,this,_1));
  m_command_pub = this->template add_publisher<sensor_msgs::JointState>("command",1);

  eu::resize(m_target_pos, this->nAx());
  eu::resize(m_target_vel, this->nAx());
  eu::resize(m_target_eff, this->nAx());

  m_target_pos = this->getPosition();
  eu::setZero(m_target_vel);
  eu::setZero(m_target_eff);

  rosdyn::VectorXd speed_limit;
  eu::copy(speed_limit, this->chain().getDQMax());
  std::string what;
  if(!ctrl.init(this->getControllerNh(), speed_limit,what))
  {
    CNR_ERROR(this->logger(), "Math ctrl of the PositionToVelocityController failed in initialization!\n" << what);
    CNR_RETURN_FALSE(this->logger(),
      "Math ctrl of the PositionToVelocityController failed in initialization:\n\t" + what);
  }
  CNR_WARN_COND(this->logger(), what.size()>0, what);

  m_configured = false;
  this->setPriority(this->QD_PRIORITY);
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
bool PositionToVelocityControllerBase<H,T>::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  m_target_pos = this->getPosition();
  eu::setZero(m_target_vel);
  eu::setZero(m_target_eff);
  m_configured = false;
  std::string what;
  if(!ctrl.starting(m_target_pos, m_target_vel,what))
  {
     CNR_ERROR(this->logger(),"Error in configuring the filters and controllers:");
     CNR_ERROR(this->logger(), what);
     CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
bool PositionToVelocityControllerBase<H,T>::doUpdate(const ros::Time& time, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  sensor_msgs::JointStatePtr cmd_msg=boost::make_shared<sensor_msgs::JointState>();
  cmd_msg->name=this->jointNames();
  cmd_msg->header.stamp=ros::Time::now();
  cmd_msg->position.resize( this->nAx(), 0.0);
  cmd_msg->velocity.resize( this->nAx(), 0.0);
  cmd_msg->effort.resize(   this->nAx(), 0.0);
  try
  {
    if(!m_configured)
    {
      m_target_pos = this->getPosition();
      eu::setZero(m_target_vel);
      eu::setZero(m_target_eff);
      double t=time.toSec();
      ctrl.update(time, &m_target_pos, &m_target_vel, &m_target_eff, &t, this->getPosition(), this->getVelocity());
    }
    else
    {
      ctrl.update(time, &m_target_pos, &m_target_vel, &m_target_eff, &m_last_sp_time,
                  this->getPosition(), this->getVelocity());
    }
    this->setCommandVelocity(ctrl.getVelCmd());
  }
  catch (...)
  {
    this->setCommandVelocity(0,0);
    CNR_RETURN_FALSE_THROTTLE(this->logger(), 2.0, "Exception!");
  }
  for (unsigned int idx=0;idx<this->nAx();idx++)
  {
    cmd_msg->position.at(idx)=m_target_pos(idx);
    cmd_msg->velocity.at(idx)=ctrl.getVelCmd()(idx);
  }
  this->getPublisher(m_command_pub)->publish(cmd_msg);
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

template<class H, class T>
bool PositionToVelocityControllerBase<H,T>::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  m_configured = false;
  ctrl.stopping();
  this->setCommandVelocity(0,0);
  CNR_RETURN_TRUE(this->logger());
}

template<class H, class T>
void PositionToVelocityControllerBase<H,T>::callback(const sensor_msgs::JointStateConstPtr msg)
{
  if(this->extractJoint(*msg, this->jointNames(), m_target_pos, m_target_vel, m_target_eff))
  {
    m_configured = true;
    m_last_sp_time = msg->header.stamp.toSec();
  }
  else
  {
    CNR_ERROR(this->logger(), " target message dimension is wrong");
    CNR_ERROR(this->logger(), " Joint Controlled names: " << cnr::control::to_string(this->jointNames()));
    CNR_ERROR(this->logger(), " msg received: " << *msg);
  }
  return;
}

template<class H, class T>
bool PositionToVelocityControllerBase<H,T>::extractJoint(
    const sensor_msgs::JointState msg, const std::vector<std::string>& names,
    rosdyn::VectorXd& pos, rosdyn::VectorXd& vel, rosdyn::VectorXd& eff)
{
  if(msg.position.size()!=msg.name.size())
  {
    return false;
  }

  for(size_t i=0;i<names.size(); i++)
  {
    std::vector<std::string>::const_iterator it = std::find(msg.name.begin(), msg.name.end(), names.at(i));
    if(it == msg.name.end())
    {
      return false;
    }

    size_t iJoint = std::distance(msg.name.begin(), it);
    eu::at(pos,i) = msg.position.at(iJoint);
    eu::at(vel,i) = msg.velocity.size() == msg.name.size() ? msg.velocity.at(iJoint) : 0 ;
    eu::at(eff,i) = msg.effort.size()   == msg.name.size() ? msg.effort.at(iJoint)   : 0 ;
  }

  return true;
}

































/**
 * @brief PositionToVelocityControllerFfw::doInit
 * @return
 */

bool PositionToVelocityControllerFfw::doInit()
{
  CNR_TRACE_START(this->logger());
  if(!this->PositionToVelocityControllerFfwBase::doInit())
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}


bool PositionToVelocityControllerFfw::doStarting(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(!this->PositionToVelocityControllerFfwBase::doStarting(time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  CNR_RETURN_TRUE(this->logger());
}


bool PositionToVelocityControllerFfw::doStopping(const ros::Time& time)
{
  CNR_TRACE_START(this->logger());
  if(!this->PositionToVelocityControllerFfwBase::doStopping(time))
  {
    CNR_RETURN_FALSE(this->logger());
  }
  this->setCommandVelocity(0,0);
  this->setCommandEffort(0,0);
  CNR_RETURN_TRUE(this->logger());
}


bool PositionToVelocityControllerFfw::doUpdate(const ros::Time& time, const ros::Duration& period)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    if(this->PositionToVelocityControllerFfwBase::doUpdate(time, period))
    {
      CNR_RETURN_FALSE(this->logger());
    }
    this->setCommandPosition(this->ctrl.getPosCmd());
    this->setCommandVelocity(this->ctrl.getVelCmd());
    this->setCommandEffort(this->ctrl.getEffCmd());
  }
  catch (...)
  {
    auto zero = this->ctrl.getVelCmd();
    eu::setZero(zero);
    this->setCommandVelocity(zero);
    this->setCommandEffort(zero);
    CNR_RETURN_FALSE_THROTTLE(this->logger(), 2.0,"something wrong: Controller '"+this->getControllerNamespace()+"'");
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

}  // namespace control
}  // namespace cnr
