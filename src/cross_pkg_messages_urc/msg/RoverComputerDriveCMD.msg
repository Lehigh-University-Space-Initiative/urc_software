# This is the message sent to the pi from drive controller
std_msgs/Header header
# Values should be normalized signed values e.x [-1,1]
# components are [Lfront, LCenter, LBack]
geometry_msgs/Vector3 cmd_l

# components are [Rfront, RCenter, RBack]
geometry_msgs/Vector3 cmd_r