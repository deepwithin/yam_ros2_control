# Aggregate status for all motors in a robot chain
std_msgs/Header header

MotorFeedback[] motors       # Array of motor feedback
uint8 num_motors             # Total number of motors
bool all_motors_healthy      # True if no motors have errors
float32 max_temperature      # Highest temperature across all motors
