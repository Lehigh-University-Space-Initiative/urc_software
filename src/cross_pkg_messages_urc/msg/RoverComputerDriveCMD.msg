# This is the message sent to the pi from drive controller
std_msgs/Header header
# Values should be normalized signed values e.x [-1,1]
# components are [Lfront, LCenter, LBack]
geometry_msgs/Vector3 cmd_l

# components are [Rfront, RCenter, RBack]
geometry_msgs/Vector3 cmd_r

# base motor
float64 cmd_b

# shoulder motor
float64 cmd_s

# elbow motor
float64 cmd_e

# components are [WRoll, WPitch, WYaw]
geometry_msgs/Vector3 cmd_w
