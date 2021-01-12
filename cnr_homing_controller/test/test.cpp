/*
nr_controller_interface::/cnr::control::/g
 *
nr_controller_interface::/cnr::control::/g
 *
nr_controller_interface::/cnr::control::/g
 *
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
 *  are met:
 *
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
nr_controller_interface::/cnr::control::/g
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <cnr_fake_hardware_interface/cnr_fake_robot_hw.h>
#include <cnr_homing_controller/cnr_homing_controller.h>

std::shared_ptr<ros::NodeHandle> root_nh;
std::shared_ptr<ros::NodeHandle> robot_nh;
std::shared_ptr<ros::NodeHandle> ctrl_nh;
std::shared_ptr<cnr_hardware_interface::FakeRobotHW> robot_hw;
std::shared_ptr<cnr::control::HomingController> ctrl;
std::shared_ptr<cnr::control::HomingController6> ctrl6;

TEST(TestSuite, Constructor)
{
  EXPECT_NO_FATAL_FAILURE(ctrl.reset(new cnr::control::HomingController()));
  EXPECT_FALSE(ctrl->init(robot_hw->get<hardware_interface::PositionJointInterface>(), *root_nh, *robot_nh));
  EXPECT_TRUE(ctrl->init(robot_hw->get<hardware_interface::PositionJointInterface>(), *robot_nh, *ctrl_nh));
}

//TEST(TestSuite, Constructor6)
//{
//  EXPECT_NO_FATAL_FAILURE(ctrl6.reset(new cnr::control::HomingController6()));
//  EXPECT_FALSE(ctrl6->init(robot_hw->get<hardware_interface::JointCommandInterface>(), *root_nh, *robot_nh));
//  EXPECT_TRUE(ctrl6->init(robot_hw->get<hardware_interface::JointCommandInterface>(), *robot_nh, *ctrl_nh));
//}

TEST(TestSuite, Desctructor)
{
  EXPECT_NO_FATAL_FAILURE(ctrl.reset());
  EXPECT_NO_FATAL_FAILURE(robot_hw.reset());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");
  root_nh  .reset(new ros::NodeHandle("/"));
  robot_nh .reset(new ros::NodeHandle("/ur10_hw"));
  ctrl_nh  .reset(new ros::NodeHandle("/ur10_hw/fake_controller"));
  robot_hw.reset(new cnr_hardware_interface::FakeRobotHW());
  robot_hw->init(*root_nh, *robot_nh);
  return RUN_ALL_TESTS();
}
