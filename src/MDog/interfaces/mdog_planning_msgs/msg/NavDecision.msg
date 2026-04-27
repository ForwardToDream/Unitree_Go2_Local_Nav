uint8 ACCEPTED=0
uint8 ADJUSTED=1
uint8 REJECTED=2

std_msgs/Header header
uint8 intent_handling
geometry_msgs/Twist cmd_vel
string reason
