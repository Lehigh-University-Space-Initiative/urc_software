# GNSS (GPS) fix
#
# Publisher:  fake_gps_node.py (navigation_urc, simulation only); no real GPS driver exists yet
# Subscriber: WaypointFollower (navigation_urc), LUSIVisionStreamer (base station telemetry)
#
# Topic: /gps_data

std_msgs/Header header

# Fix status (the simulator always sends 1; meaning is up to the real driver)
uint8 status

# Position: x = latitude (degrees), y = longitude (degrees), z = altitude
geometry_msgs/Vector3 lla

# Ground speed
float64 speed

# Course over ground: compass heading of travel in degrees (0 = north, clockwise)
float64 course

# Number of satellites used in the fix
int32 sats

# Position accuracy for each lla component
geometry_msgs/Vector3 lla_acc
