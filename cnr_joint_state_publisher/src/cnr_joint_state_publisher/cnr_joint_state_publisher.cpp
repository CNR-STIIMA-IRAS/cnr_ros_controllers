#include <cnr_joint_state_publisher/cnr_joint_state_publisher.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::JointStatePublisher, 
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(cnr::control::MultiChainStatePublisher, 
                       controller_interface::ControllerBase)
namespace  cnr
{
namespace control
{

//!
//! \brief JointStatePublisher::~JointStatePublisher
//!
JointStatePublisher::~JointStatePublisher()
{
  CNR_TRACE_START(this->logger());
  if(this->getPublisher(m_pub_handle))
  {
    this->getPublisher(m_pub_handle)->shutdown();
  }

  if (!this->isStopped())
  {
    this->stopping(ros::Time::now());
  }
}

//!
//! \brief JointStatePublisher::doInit
//! \return
//!
bool JointStatePublisher::doInit()
{
  CNR_TRACE_START(this->logger());
  if (this->nAx()==0)
  {
    CNR_RETURN_FALSE(this->logger(), "The number of controlled axes is 0. Check the configuration. Abort");
  }
  m_pub_handle = this->template add_publisher<sensor_msgs::JointState>("joint_states", 1);

  if(!this->getPublisher(m_pub_handle))
  {
    CNR_FATAL(this->logger(), "Failed in creating the publisher 'joint_states '");
    CNR_RETURN_TRUE(this->logger());
  }

  CNR_TRACE(this->logger(), "Published Topic '" + this->getPublisher(m_pub_handle)->getTopic()
                     + "', axis names: " + cnr::control::to_string(this->jointNames())
            + " n. axes: " + std::to_string(this->nAx()));

  CNR_RETURN_TRUE(this->logger());
}

//!
//! \brief JointStatePublisher::doStarting
//! \return
//!
bool JointStatePublisher::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  CNR_RETURN_TRUE(this->logger());
}

//!
//! \brief JointStatePublisher::doUpdate
//! \return
//!
bool JointStatePublisher::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    sensor_msgs::JointStatePtr  msg;
    msg.reset(new sensor_msgs::JointState());
    msg->position.resize(this->nAx(), 0);
    msg->velocity.resize(this->nAx(), 0);
    msg->effort.resize(this->nAx(), 0);
    msg->name = this->jointNames();

    for(std::size_t iAx = 0; iAx<this->chain().getActiveJointsNumber(); iAx++)
    {
      msg->name    .at(iAx) = this->chain().getActiveJointName(iAx);
      msg->position.at(iAx) = this->getPosition(iAx);
      msg->velocity.at(iAx) = this->getVelocity(iAx);
      msg->effort  .at(iAx) = this->getEffort(iAx);
    }
    msg->header.stamp = ros::Time::now();
    if(!this->publish(m_pub_handle,msg))
    {
      CNR_TRACE(this->logger(), "The publisher " + std::to_string(m_pub_handle) + "failed" );
      CNR_RETURN_FALSE(this->logger());
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(this->logger(), "Exception caught" + std::string(e.what()));
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

//!
//! \brief JointStatePublisher::doStopping
//! \return
//!
bool JointStatePublisher::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  CNR_RETURN_TRUE(this->logger()); 
}


/**
 * @brief 
 * 
 * 
 * 
 * 
 * 
 */
//!
//! \brief MultiChainStatePublisher::~MultiChainStatePublisher
//!
MultiChainStatePublisher::~MultiChainStatePublisher()
{
  CNR_TRACE_START(this->logger());
  if(this->getPublisher(m_pub_handle))
  {
    this->getPublisher(m_pub_handle)->shutdown();
  }

  if (!this->isStopped())
  {
    this->stopping(ros::Time::now());
  }
}

//!
//! \brief MultiChainStatePublisher::doInit
//! \return
//!
bool MultiChainStatePublisher::doInit()
{
  CNR_TRACE_START(this->logger());
  for(auto const & id : this->chainNames() )
  {
    if (this->nAx(id)==0)
    {
      CNR_RETURN_FALSE(this->logger(), "The number of controlled axes is 0. Check the configuration. Abort");
    }
    m_pub_handle = this->template add_publisher<sensor_msgs::JointState>("joint_states", 1);

    if(!this->getPublisher(m_pub_handle))
    {
      CNR_FATAL(this->logger(), "Failed in creating the publisher 'joint_states '");
      CNR_RETURN_TRUE(this->logger());
    }

    CNR_TRACE(this->logger(), "Published Topic '" + this->getPublisher(m_pub_handle)->getTopic()
                      + "', axis names: " + cnr::control::to_string(this->jointNames(id))
              + " n. axes: " + std::to_string(this->nAx(id)));
  }
  CNR_RETURN_TRUE(this->logger());
}

//!
//! \brief MultiChainStatePublisher::doStarting
//! \return
//!
bool MultiChainStatePublisher::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger());
  CNR_RETURN_TRUE(this->logger());
}

//!
//! \brief MultiChainStatePublisher::doUpdate
//! \return
//!
bool MultiChainStatePublisher::doUpdate(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  try
  {
    sensor_msgs::JointStatePtr  msg;
    msg.reset(new sensor_msgs::JointState());
    msg->name.clear();
    msg->position.clear();
    msg->velocity.clear();
    msg->effort.clear();

    for(const auto & id : this->chainNames() )
    {
      for(std::size_t iAx = 0; iAx<this->chain(id).getActiveJointsNumber(); iAx++)
      {
        msg->name    .push_back( this->chain(id).getActiveJointName(iAx) );
        msg->position.push_back( this->getPosition(id,iAx) );
        msg->velocity.push_back( this->getVelocity(id,iAx) );
        msg->effort  .push_back( this->getEffort(id,iAx) );
      }
    }
    msg->header.stamp = ros::Time::now();
    if(!this->publish(m_pub_handle,msg))
    {
      CNR_TRACE(this->logger(), "The publisher " + std::to_string(m_pub_handle) + "failed" );
      CNR_RETURN_FALSE(this->logger());
    }
  }
  catch (std::exception& e)
  {
    CNR_RETURN_FALSE(this->logger(), "Exception caught" + std::string(e.what()));
  }
  CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());
}

//!
//! \brief JointStatePublisher::doStopping
//! \return
//!
bool MultiChainStatePublisher::doStopping(const ros::Time& time)
{
  unused(time);
  CNR_TRACE_START(this->logger());
  CNR_RETURN_TRUE(this->logger()); 
}

}
}


