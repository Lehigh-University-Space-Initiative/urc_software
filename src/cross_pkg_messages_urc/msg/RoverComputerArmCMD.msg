# Arm joint values for the six joints, used in both directions
#
# /roverArmCommands: joint position TARGETS, published by MockArmHardware (main computer), read by ArmMotorManager (arm Pi)
# /roverArmPos:      measured joint POSITIONS, published by ArmMotorManager, read by MockArmHardware
#
# Units: radians (MockArmHardware flips the sign of the elbow and wrist 2 between MoveIt and the arm Pi)

std_msgs/Header header

# Base rotation joint
float64 cmd_b

# Shoulder joint
float64 cmd_s

# Elbow joint
float64 cmd_e

# Wrist joints: x = wrist 1, y = wrist 2, z = wrist 3 (originally described as roll, pitch, yaw)
geometry_msgs/Vector3 cmd_w
