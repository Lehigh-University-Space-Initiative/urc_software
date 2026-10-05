# Raw operator input from the SpaceMouse, after deadband and shaping
#
# Publisher:  SpaceMouseMapper (base station) on /armInputRaw
# Subscriber: ArmMotorManager (arm Pi, uses only the buttons for the gripper), the GUI's Telemetry panel
#
# Units: each axis is a unitless value in [-1, 1]

std_msgs/Header header

# Translation input: x, y, z
geometry_msgs/Vector3 linear_input

# Rotation input: x, y, z
geometry_msgs/Vector3 angular_input

# Gripper buttons: nonzero while held (SpaceMouseMapper sends 0.01); left opens, right closes
float64 left_btn
float64 right_btn
