# URC 2027 drivetrain command
#
# Four independently powered wheels and two steerable front wheels.
# Wheel velocities are in rad/s at the wheel, not motor shaft.
# Steering targets are in radians relative to calibrated straight-ahead.
# Positive steering angles follow the ROS convention (left turn).
#
# This message describes requested actuator motion, not measured feedback.
# It does not authorize hardware output by itself.

std_msgs/Header header

# Drive wheel angular velocities (rad/s)
float64 front_left_velocity
float64 rear_left_velocity
float64 front_right_velocity
float64 rear_right_velocity

# Front steering position targets (rad)
float64 front_left_steering
float64 front_right_steering
