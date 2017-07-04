#-*- coding:utf-8 -*-
# AUTHOR:   Brandon Kallaher
# FILE:     control_wrapper.py
# CREATED:  2017-07-03 23:20:06
# MODIFIED: 2017-07-03 23:54:22
# DESC:     This class is used to wrap around the control messages for
#           readability and simplicity. This uses a singleton so that all
#           instances have the same internal state at any given time and
#           therefore making relative commands possible from any instance

from robosub.msg import control

# Metaclass for singleton definition
class SingletonType(type):
    def __call__(cls, *args, **kwargs):
        try:
            return cls.__instance
        except AttributeError:
            cls.__instance = super(SingletonType, cls).__call__(*args, **kwargs)
            return cls.__instance

class control_wrapper():
    __metaclass__ = SingletonType
    def __init__(self):
        self.control_msg = control()

    # Allow a user to get the current state
    def getCurrentState(self):
        return self.control_msg

    # Yaw state-value pair setter
    def setYawLeft(self, state, value):
        self.control_msg.yaw_state = state
        self.control_msg.yaw_left = value

    # Pitch state-value pair setter
    def setPitchDown(self, state, value):
        self.control_msg.pitch_state = state
        self.control_msg.pitch_down = value

    # Roll state-value pair setter
    def setRollRight(self, state, value):
        self.control_msg.roll_state = state
        self.control_msg.roll_right = value

    # Dive state-value pair setter
    def setDive(self, state, value):
        self.control_msg.dive_state = state
        self.control_msg.dive = value

    # Forward state-value pair setter
    def setForward(self, state, value):
        self.control_msg.forward_state = state
        self.control_msg.forward = value

    # Strafe state-value pair setter
    def setStrafeLeft(self, state, value):
        self.control_msg.strafe_state = state
        self.control_msg.strafe_left = value

    # Allow for clearing of the movement state
    def clearState(self):
        self.control_msg = control()







# Main for testing
if __name__ == "__main__":
    c = control_wrapper()
    print c
    print c.getCurrentState()
    c.setRollRight(control.STATE_ABSOLUTE, 50)
    c.setDive(control.STATE_ERROR, 10)
    c.setForward(control.STATE_RELATIVE, 3)
    c.setYawLeft(control.STATE_ERROR, 20)
    c.setPitchDown(control.STATE_ABSOLUTE, 45)
    c.setStrafeLeft(control.STATE_RELATIVE, 145)
    d = control_wrapper()
    print d
    print d.getCurrentState()
    d.clearState()
    print c.getCurrentState()
