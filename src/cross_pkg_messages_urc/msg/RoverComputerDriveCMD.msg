# This is the message sent to the pi from drive controller
std_msgs/Header header
# Values should be normalized signed values e.x [-1,1]
# lf=left front, lb=left back, ls=left steering (front wheel)
float64 lf
float64 lb
float64 ls

# rf=right front, rb=right back, rs=right steering (front wheel)
float64 rf
float64 rb
float64 rs