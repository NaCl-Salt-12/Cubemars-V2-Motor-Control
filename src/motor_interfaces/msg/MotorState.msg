# This message contains data used to describe the current state of CubeMars v2 motors

# The state of each motor is defined by:
#   - position of the motor (rad)
#   - absolute position of the motor (rad)
#   - velocity of the motor (rad/s)
#   - current applied (A)
#   - torque applied (nm)
#   - temperature (C)
#
# Each motor is uniquely identified by its name 

string name
float32 position
float32 abs_position 
float32 velocity
float32 torque
float32 current
int32 temperature

