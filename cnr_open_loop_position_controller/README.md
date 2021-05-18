# Open loop position controller
*OpenLoopPositionController* subscribes a joint_state topic and applies it as joint_command

```yaml
  ctrl1:
    type:        cnr/control/OpenLoopPositionController
    setpoint_topic_name: "/ur5/joint_target"  # joint command topic (type: sensor_msgs/JointState)
    controlled_joints: all  # list of joint names, or 'all' to use all the joint managed by the RobotHW
    enable_setpoint_watchdog: false # optional (default: true). watchdog time for setpoint_topic
```