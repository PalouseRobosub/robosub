#Description
#
# This message type is used for for updating the control system goals and
# displaying the current goals of the control system.  The control system has 6
# possible goals. Forward, strafe, and dive are position goals, while roll,
# pitch, and yaw are orientation goals. The control system internally has
# variables that store its current goals. Sending a message of this type will
# update those internal goals.

# When adjusting these goals, one must fill out the associated "state" variable
# for that goal, which determines how the goal is changed.
#   STATE_NONE means the control system will not update this particular goal.
#   STATE_ABSOLUTE means the control system will set its internal goal to be
#       the new value.
#   STATE_RELATIVE means the control system will set its internal goal to be
#       the current state plus the new value (i.e. if the current roll is 15
#       degrees and we send a relative goal of 10 degrees, the control system's
#       new goal will be 25 degrees).
#   STATE_ERROR means the control system will not use feedback, this allows us
#       to directly control the error signal going into the PID. Generally,
#       don't use this state if you didn't understand the previous sentence.


# When moving forward/backward or strafing, we cannot use absolute or relative
# goals, we must always use STATE_ERROR. This is because we currently do not
# know where the sub is in the pool, therefore the control system cannot use
# feedback to control x,y position in the pool.

# When the control system is publishing its current state, it will always be
# either STATE_ABSOLUTE or STATE_ERROR. Recall that the other two states are
# just used when updating the control system's internal state.

# Examples:
# To set the sub to be at 1 meter depth:
#   forward_state, strafe_state, roll_state, pitch_state, yaw_state=STATE_NONE
#   dive_state = STATE_ABSOLUTE
#   dive = -1
# To set the sub to be 1 meter deeper than its current depth:
#   forward_state, strafe_state, roll_state, pitch_state, yaw_state=STATE_NONE
#   dive_state = STATE_RELATIVE
#   dive = -1
# To make the sub move forward:
#   dive_state, strafe_state, roll_state, pitch_state, yaw_state = STATE_NONE
#   forward_state = STATE_ERROR
#   forward = 1 (larger/smaller numbers will make sub move faster/slower)


#control states
uint8 forward_state
uint8 strafe_state
uint8 dive_state
uint8 roll_state
uint8 pitch_state
uint8 yaw_state

#control commands
float64 forward
float64 strafe_left
float64 dive
float64 roll_right
float64 pitch_down
float64 yaw_left


#Enums for state variables
uint8 STATE_NONE     = 0
uint8 STATE_ABSOLUTE = 1
uint8 STATE_RELATIVE = 2
uint8 STATE_ERROR    = 3


