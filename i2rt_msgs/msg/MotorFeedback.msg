# Individual DM motor feedback data
# Provides detailed status for each motor in the robot chain

std_msgs/Header header

uint8 motor_id              # Motor ID on CAN bus (1-7 for YAM arm)
string motor_type           # Motor type: DM4340, DM4310, DM3507

# Position and velocity
float32 position            # Joint position in radians
float32 velocity            # Joint velocity in rad/s
float32 torque              # Motor torque in Nm

# Temperatures
float32 mos_temperature     # MOS (MOSFET) temperature in Celsius
float32 rotor_temperature   # Rotor temperature in Celsius

# Error status
uint8 error_code            # Raw DM motor error nibble (see constants below)
bool has_error              # True if error_code != ERROR_NORMAL
string error_description    # Human-readable error description

# Error code definitions — these are the DM motor's actual 4-bit status
# codes (i2rt_can_driver::motor_error_code), not an invented bitmask.
uint8 ERROR_DISABLED = 0
uint8 ERROR_NORMAL = 1
uint8 ERROR_OVER_VOLTAGE = 8
uint8 ERROR_UNDER_VOLTAGE = 9
uint8 ERROR_OVER_CURRENT = 10
uint8 ERROR_MOSFET_OVER_TEMPERATURE = 11
uint8 ERROR_MOTOR_OVER_TEMPERATURE = 12
uint8 ERROR_LOSS_COMMUNICATION = 13
uint8 ERROR_OVERLOAD = 14
