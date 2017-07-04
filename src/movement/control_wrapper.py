#-*- coding:utf-8 -*-
# AUTHOR:   Brandon Kallaher
# FILE:     control_wrapper.py
# CREATED:  2017-07-03 23:20:06
# MODIFIED: 2017-07-03 23:45:36
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

    def getCurrentState(self):
        return self.control_msg 

    def setYawLeft(self, state, value):
        self.control_msg.yaw_state = state
        self.control_msg.yaw_left = value

    def setPitchDown(self, state, value):
        self.control_msg.pitch_state = state
        self.control_msg.pitch_down = value

    def setRollRight(self, state, value):
        self.control_msg.roll_state = state
        self.control_msg.roll_right = value



# Main for testing
if __name__ == "__main__":
    c = control_wrapper()
    print c
    print c.getCurrentState()
    c.setRollRight(control.STATE_ABSOLUTE, 50)
    d = control_wrapper()
    print d
    print d.getCurrentState()
