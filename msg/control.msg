#control states
uint8 forward_state
uint8 strafe_state
uint8 dive_state
uint8 roll_state
uint8 pitch_state
uint8 yaw_state

#control commands
float64 forward
float64 strafe_right
float64 dive
float64 roll_right
float64 pitch_up
float64 yaw_right


#Enums for state variables
uint8 STATE_NONE     = 0
uint8 STATE_ABSOLUTE = 1
uint8 STATE_RELATIVE = 2
uint8 STATE_ERROR    = 3
