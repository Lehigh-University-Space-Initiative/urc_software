# This is the message sent to the pi from drive controller
std_msgs/Header header
# Values should be normalized signed values e.x [-1,1]

# base motor
float64 cmd_b

# shoulder motor
float64 cmd_s

# elbow motor
float64 cmd_e

# components are [WRoll, WPitch, WYaw]
geometry_msgs/Vector3 cmd_w