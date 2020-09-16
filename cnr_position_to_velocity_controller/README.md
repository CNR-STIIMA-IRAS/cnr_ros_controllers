# cnr_position_to_velocity_controller#

Package to control joint position using velocity as control variable. It is a single-input-single-output controller, namely each joint has its controller.

Plugins list:

- cnr/control/PositionToVelocityControllerFfw uses a VelocityJointInterface

- cnr/control/StateSpacePositionToVelocityControllerFfw uses a PosVelEffJointInterface, namely also torque is feedforwarded. 





### parameters ###
```yaml


  your_controller_name:
    controlled_joint: "JOINT_NAME" # controlled joint
    setpoint_topic_name: "TOPIC_NAME" # name of the JointState topic with provide the position setpoint
    use_target_velocity: true # if true, the velocity feedforward (namely, the velocity field of setpoint_topic_name) is added to the controller output. if false, velocity feedforward is not used.
    antiwindup_ratio: 1.0 #antiwindup_ratio = T_t/T_i. if you don't know what you're doing keep it equal to 1.0
    
    position_maximum_error: 0.1 # maximum allowed error, controller goes in error mode if it is exceeded. Error mode will set the command velocity equal to zero.
    position_minimum_error: 0.00001 # minimum error, if the absolute value of error is smaller than this threshold, the error is set equal to 0. It is useful in case of stick-slip to reduce chattering.
    
    interpolate_setpoint: true # to enable/disable interpolation of setpoint topic if there are not new messages. The interpolation is a first-order-hold, that is: target_position=last_received_target_position+last_received_target_velocity*time_from_last_message. This option is used when the setpoint_topic has a frequency smaller than the controller rate. If disable the last value of target position is used.
    maximum_interpolation_time: 0.01 # maximum time from last message used for interpolation. If time_from_last_message is greater than maximum_interpolation_time, the target position is set as target_position=last_received_target_position+last_received_target_velocity*maximum_interpolation_time.
    
    pos_filter: {type: unity} # filter on feedback position, if you don't know what you're doing keep it disable, that is type: unity
    target_pos_filter: {type: unity} # filter on setpoint position, if you don't know what you're doing keep it disable, that is type: unity
    controller: {type: "proportional", proportional_gain: 10.0} # controller (excluding the integral part). If the lower controllers works properly, a proportional controller should be sufficient. See eigen_control_toolbox to implement advanced controllers.
    integral_controller: {type: "proportional", proportional_gain: 0.0} # controller (excluding the integral part). If the lower controllers works properly, an integral controller should not be required (set it as a constant equal to 0). See eigen_control_toolbox to implement advanced controllers.


```
