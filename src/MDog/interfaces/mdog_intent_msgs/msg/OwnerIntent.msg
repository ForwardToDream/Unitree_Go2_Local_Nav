uint8 STOP=0
uint8 FORWARD=1
uint8 BACK=2
uint8 LEFT=3
uint8 RIGHT=4
uint8 TURN_LEFT=5
uint8 TURN_RIGHT=6

std_msgs/Header header
uint8 command
float32 strength
float32 preferred_speed
float32 confidence
string source
