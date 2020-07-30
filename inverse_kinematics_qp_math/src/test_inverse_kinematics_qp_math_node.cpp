#include <inverse_kinematics_qp_math/inverse_kinematics_qp_math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <rosdyn_core/primitives.h>
#include <rosdyn_core/urdf_parser.h>
#include <rosdyn_core/frame_distance.h>
#include <rosdyn_core/urdf_parser.h>


int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "test_inverse_kinematics_qp_math");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::Rate r(125);

  urdf::Model model;
  model.initParam("robot_description");
  Eigen::Vector3d grav;
  grav << 0, 0, -9.806;
  std::string base_frame = "world";
  std::string tool_frame = "ur5_ee_link";
  std::string elbow_frame = "ur5_forearm_link";
  if (!node_handle.getParam("base_frame",base_frame) || !node_handle.getParam("tool_frame",tool_frame) || !node_handle.getParam("elbow_frame",elbow_frame))
  {
    ROS_ERROR("unable to load param base_frame || tool_frame || elbow_frame");
    return -1;
  }
  std::shared_ptr<rosdyn::Chain> chain_ee = rosdyn::createChain(model,base_frame,tool_frame,grav);
  std::shared_ptr<rosdyn::Chain> chain_elbow = rosdyn::createChain(model,base_frame,elbow_frame,grav);


  const unsigned int df=7; //chain_ee->getActiveJointsNumber();
  const unsigned int df_elbow=4; //chain_elbow->getActiveJointsNumber();
  std::vector<std::string> jointNames(df);
  jointNames.at(0) = "linear_motor_cursor_joint";
  jointNames.at(1) = "ur5_shoulder_pan_joint";
  jointNames.at(2) = "ur5_shoulder_lift_joint";
  jointNames.at(3) = "ur5_elbow_joint";
  jointNames.at(4) = "ur5_wrist_1_joint";
  jointNames.at(5) = "ur5_wrist_2_joint";
  jointNames.at(6) = "ur5_wrist_3_joint";
  chain_ee->setInputJointsName(jointNames);
  std::vector<std::string> joints_base_elbow;
  for (unsigned int idx=0;idx<df_elbow;idx++)
  {
    joints_base_elbow.push_back(jointNames.at(idx));
  }
  chain_elbow->setInputJointsName(joints_base_elbow);

  Eigen::VectorXd rest_config(df);
  rest_config<< 1,-2,1,2,-1,2,0;
  Eigen::VectorXd jointState(df);
  jointState=rest_config;

  Eigen::VectorXd jointUB(df);
  Eigen::VectorXd jointLB(df);
  Eigen::VectorXd velUB(df);
  Eigen::VectorXd accUB(df);

  jointUB.setConstant(3);
  jointLB.setConstant(-3);
  velUB.setConstant(2);
  accUB.setConstant(100);

  ROS_WARN_STREAM("position upper bound = " << jointUB.transpose());
  ROS_WARN_STREAM("position lower bound = " << jointLB.transpose());
  ROS_WARN_STREAM("velocity limits = " << velUB.transpose());
  ROS_WARN_STREAM("acceleration limits=" << accUB.transpose());

  double st=8e-3; // [ms]
  double lambda_distance=1;
  double lambda_effort=1e-2;
  double lambda_return=1e-2;
  double gain_clik=1.0;

  if (!node_handle.getParam("gain_clik",gain_clik) || !node_handle.getParam("lambda_effort",lambda_effort) || !node_handle.getParam("lambda_return",lambda_return))
  {
    ROS_ERROR("unable to load param: gain_clik || lambda_effort");
    return -1;
  }
  std::vector<int> select_task_axis;
  if (!node_handle.getParam("select_task_axis",select_task_axis))
  {
    ROS_ERROR("unable to load param: select_task_axis");
    return -1;
  }
  if (select_task_axis.size()!=6)
  {
    ROS_ERROR("length of param 'select_task_axis' must must be equal to 6");
    return -1;
  }

  double clearance_threshold=1;
  if (!node_handle.getParam("clearance_threshold",clearance_threshold))
  {
    ROS_ERROR("unable to load param: clearance_threshold");
    return -1;
  }
  std::string secondary_task;
  if (!node_handle.getParam("secondary_task",secondary_task))
  {
    ROS_ERROR("unable to load param: secondary_task");
    return -1;
  }

  double distance=1e6;
  Eigen::VectorXd task_error_in_b_full;
  Eigen::MatrixXd I_df(df,df);
  I_df.setIdentity();

  Eigen::VectorXd obstacle_in_b(3);
  obstacle_in_b(0)=1.5;
  obstacle_in_b(1)=-0.5;
  obstacle_in_b(2)=-0.5;
  Eigen::Affine3d obstacle_pose_in_b;
  obstacle_pose_in_b.linear().setIdentity();
  obstacle_pose_in_b.translation()=obstacle_in_b;

  Eigen::Affine3d Tbt=chain_ee->getTransformation(rest_config); // base <- target
  Tbt.translation()(1)+=0.05;
  Eigen::Vector6d targetDx;
  targetDx.setZero();

  rosdyn::getFrameDistance(Tbt,chain_ee->getTransformation(jointState),task_error_in_b_full);

  ROS_INFO_STREAM("q = " << jointState.transpose());
  ROS_INFO_STREAM("x = \n" << Tbt.matrix());
  ROS_INFO_STREAM("cartesian error= " << task_error_in_b_full);

  Eigen::VectorXd jointVel(df);
  jointVel.setZero(df);
  Eigen::VectorXd jointVelOld(df);
  jointVelOld=jointVel;
  Eigen::VectorXd jointStateOld(df);
  jointStateOld=jointState;

  inverse_kinematics_qp::math::InverseKinematicsQP local_solver;

  local_solver.setAxisNumberTask1(df);
  local_solver.setDynamicsChainTask1(chain_ee);
  local_solver.setSamplingPeriod(st);
  local_solver.setInitialState(jointState,jointVel);
  local_solver.computeTaskSelectionMatrix(select_task_axis);
  local_solver.setSecondaryTask(secondary_task);
  local_solver.setClearanceOptions(chain_elbow,df_elbow,clearance_threshold,rest_config, obstacle_pose_in_b);
  local_solver.setConstraints(jointUB,jointLB,velUB,accUB);
  local_solver.setWeigthFunction(lambda_distance,lambda_effort,lambda_return,gain_clik);
  local_solver.updateMatrices();

  Eigen::VectorXd next_vel(df);
  next_vel.setZero();
//  local_solver.computedCostrainedSolution(targetDx,Tba,jointState,jointVel,next_vel);
//  local_solver.computedUncostrainedSolution(targetDx,Tba,jointState,jointVel,next_vel);

  Eigen::Affine3d Tbe=chain_elbow->getTransformation(jointState.head(df_elbow));
  distance=sqrt((obstacle_in_b-Tbe.translation()).transpose() * (obstacle_in_b-Tbe.translation()) );
  ROS_INFO_STREAM("x_elbow=" << Tbe.translation());

  for (unsigned int idx=0;idx<1000;idx++)
  {
    jointState=local_solver.getJointPosition();
    jointVel=local_solver.getJointVelocity();

    Eigen::Affine3d Tba=chain_ee->getTransformation(jointState);
    ROS_INFO_STREAM_ONCE("cart_error=" << task_error_in_b_full.transpose());

    rosdyn::getFrameDistance(Tbt,Tba,task_error_in_b_full);
    local_solver.computedCostrainedSolution(targetDx,Tbt,jointState,jointVel,next_vel);
    local_solver.updateState(next_vel);
    if (idx%20==0)
    {
      Tbe=chain_elbow->getTransformation(jointState.head(df_elbow));
      distance=sqrt( (obstacle_in_b-Tbe.translation()).transpose() * (obstacle_in_b-Tbe.translation()) );
      ROS_INFO_STREAM("cart_error=" << task_error_in_b_full.transpose());
      //ROS_INFO("Norm=%f",jointState.squaredNorm());
      ROS_INFO("Distance=%f",distance);
    }
    if (idx==0)
      local_solver.printALL();
  }



  return 0;
//  while(ros::ok())
//  {
//    Eigen::Affine3d Tba=chain_ee->getTransformation(jointState); // current ee pose
//    // Eigen::Vector3d Oa_in_b=Tba.translation(); // current ee position
//    Eigen::MatrixXd jacobian_ee = select_task_axis_matrix*(chain_ee->getJacobian(jointState));

//    rosdyn::getFrameDistance(Tbt,Tba,task_error_in_b_full);

//    task_error_in_b = select_task_axis_matrix*task_error_in_b_full;



//    if (iter%100==0)
//    {
//      ROS_INFO_STREAM("q = " << jointState.transpose());
//      ROS_INFO_STREAM("x = \n" << Tba.matrix());
//      ROS_INFO_STREAM("jointVel = " << jointVel.transpose());
//      ROS_INFO("task_error_in_b=%f \n distance=%f",task_error_in_b.norm(),distance);
//    }

//    jointStateOld=jointState;
//    jointVelOld=jointVel;

//    Eigen::MatrixXd hessian(df,df);
//    Eigen::VectorXd linear(df);

//    hessian=lambda_effort*I_df;
//    linear.setZero();

//    if (!secondary_task.compare("minimum_velocity_norm"))
//    {
//      ROS_WARN_ONCE("Secondary objective for redundancy: minimum_velocity_norm");
//      hessian=I_df;
//      linear.setZero();
//    }
//    else if (!secondary_task.compare("clearance"))
//    {
//      ROS_WARN_ONCE("Secondary objective for redundancy: clearance");
//      Eigen::Affine3d Tbe=chain_elbow->getTransformation(jointState.head(df_elbow)); // current elbow pose
//      Eigen::Vector3d Oelbow_in_b=Tbe.translation(); // current elbow position
//      Eigen::MatrixXd jacobian_elbow(3,df);
//      jacobian_elbow << (chain_elbow->getJacobian(jointState.head(df_elbow))).block(0,0,3,df_elbow), Eigen::MatrixXd::Zero(3,df-df_elbow);

//      marker = sub_markers.getData();
//      if (marker.markers.size()>0)
//      {
//        obstacle_in_b(0)=marker.markers[0].pose.position.x;
//        obstacle_in_b(1)=marker.markers[0].pose.position.y;
//        obstacle_in_b(2)=marker.markers[0].pose.position.z;
//        if (marker.markers.size()>1)
//        {
//          for (unsigned int idx=1;idx<marker.markers.size();idx++)
//          {
//            Eigen::VectorXd candidateMarker(3);
//            candidateMarker(0)=marker.markers.at(idx).pose.position.x;
//            candidateMarker(1)=marker.markers.at(idx).pose.position.y;
//            candidateMarker(2)=marker.markers.at(idx).pose.position.z;
//            if ( (Oelbow_in_b-candidateMarker).norm() < (Oelbow_in_b-obstacle_in_b).norm() )
//                obstacle_in_b=candidateMarker;
//          }
//        }
//      }
//      else
//      {
//        obstacle_in_b(0)=2*clearance_threshold;
//        obstacle_in_b(1)=2*clearance_threshold;
//        obstacle_in_b(2)=2*clearance_threshold;
//      }
//      distance= (Oelbow_in_b-obstacle_in_b).norm();
//      if (distance<=clearance_threshold)
//        lambda_distance=1.0;
//      else
//        lambda_distance=0.0;

//      hessian+=lambda_distance*(-st*(jacobian_elbow.transpose() * jacobian_elbow));
//      linear+=lambda_distance*(-jacobian_elbow.transpose()*(Oelbow_in_b-obstacle_in_b) );
//      hessian+=lambda_return*st*I_df;
//      linear+=lambda_return*(jointState-rest_config);
//    }
//    else
//    {
//      ROS_WARN_ONCE("Undefined secondary objective for redundancy: using joint range availability as default");
//      Eigen::VectorXd mean_config=0.5*(jointUB+jointLB);
//      hessian+=10.0*lambda_return*st*I_df;
//      linear+=10.0*lambda_return*(jointState-mean_config);
//    }

//    Eigen::MatrixXd CE=jacobian_ee.transpose();
//    Eigen::VectorXd ce0=-(gain_clik*task_error_in_b);
//    Eigen::MatrixXd CI(df,6*df);
//    CI << st*I_df, -st*I_df, I_df, -I_df, I_df, -I_df; // pos, vel, acc limits
//    Eigen::VectorXd ci0(6*df);
//    ci0 << -(jointLB-jointState), jointUB-jointState, -velLB, velUB, -(st*accLB+jointVel), jointVel+st*accUB; // pos, vel, acc limits

//    Eigen::solve_quadprog(hessian, linear, CE, ce0, CI, ci0, jointVel);
//    jointState+=st*jointVel;

//    for (unsigned int idx=0;idx<df;idx++)
//    {
//      if (jointState(idx)>jointUB(idx)+1e-4 || jointVel(idx)>velUB(idx)+1e-4 || (jointVel(idx)-jointVelOld(idx))/st>accUB(idx)+1e-4 )
//        ROS_WARN("Iter %d: Exceeded kinematic limits joint %d: pos: %f, vel: %f, acc: %f", iter, idx, jointState(idx), jointVel(idx), (jointVel(idx)-jointVelOld(idx))/st );
//      if (jointState(idx)<jointLB(idx)-1e-4 || jointVel(idx)<velLB(idx)-1e-4 || (jointVel(idx)-jointVelOld(idx))/st<=accLB(idx)-1e-4 )
//        ROS_WARN("Iter %d: Exceeded kinematic limits joint %d: pos: %f, vel: %f, acc: %f", iter, idx, jointState(idx), jointVel(idx), (jointVel(idx)-jointVelOld(idx))/st );
//    }


//    for (unsigned int idx=0;idx<df;idx++)
//      joint_state_msg.position.at(idx)=jointState(idx);
//    joint_state_msg.header.stamp=ros::Time::now();
//    joint_pos_pub.publish(joint_state_msg);

//    iter++;

//    r.sleep();
//  }
}
