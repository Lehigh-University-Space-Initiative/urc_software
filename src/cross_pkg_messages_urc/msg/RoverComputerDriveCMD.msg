# Per-wheel drive command for the six wheels (three per side)
#
# Publisher:  DriveTrainManager (main computer) on /roverDriveCommands
# Subscriber: MotorCtr_node (driveline Pi), LUSIVisionStreamer (base station telemetry)
#
# Units: each value is wheel angular velocity in rad/s (MotorCtr_node divides by 20 to get motor power)
# The original design called for normalized values in [-1, 1]; the code sends rad/s instead

std_msgs/Header header

# Left side: x = front, y = middle (center), z = back
geometry_msgs/Vector3 cmd_l

# Right side: x = front, y = middle (center), z = back
geometry_msgs/Vector3 cmd_r
