# -*- coding:utf-8 -*-
# AUTHOR:   Brandon Kallaher
# FILE:     control_wrapper.py
# CREATED:  2017-07-03 23:20:06
# MODIFIED: 2017-07-04 15:16:49
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
        self._control_msg = control()

    # Allow a user to get the current state
    def getCurrentState(self):
        return self._control_msg

    # Allow for clearing of the movement state
    def clearState(self):
        self._control_msg = control()
        
    # ---------------Yaw Methods--------------------

    # Yaw state-value pair setter
    def setYawLeft(self, state, value):
        self._control_msg.yaw_state = state
        self._control_msg.yaw_left = value

    def yawAbsolute(self, value):
        self.setYawLeft(control.STATE_ABSOLUTE, value)

    def yawRelative(self, value):
        self.setYawLeft(control.STATE_RELATIVE, value)

    def yawError(self, error):
        self.setYawLeft(control.STATE_ERROR, error)

    # ---------------Pitch Methods------------------
    
    # Pitch state-value pair setter
    def setPitchDown(self, state, value):
        self._control_msg.pitch_state = state
        self._control_msg.pitch_down = value

    def pitchAbsolute(self, value):
        self.setPitchDown(control.STATE_ABSOLUTE, value)

    def pitchRelative(self, value):
        self.setPitchDown(control.STATE_RELATIVE, value)

    def pitchError(self, error):
        self.setPitchDown(control.STATE_ERROR, error)

    # ---------------Roll Methods-------------------

    # Roll state-value pair setter
    def setRollRight(self, state, value):
        self._control_msg.roll_state = state
        self._control_msg.roll_right = value

    def rollAbsolute(self, value):
        self.setRollRight(control.STATE_ABSOLUTE, value)

    def rollRelative(self, value):
        self.setRollRight(control.STATE_RELATIVE, value)

    def rollError(self, error):
        self.setRollRight(control.STATE_ERROR, error)

    # ---------------Dive Methods-------------------

    # Dive state-value pair setter
    def setDive(self, state, value):
        self._control_msg.dive_state = state
        self._control_msg.dive = value

    def diveAbsolute(self, value):
        self.setDive(control.STATE_ABSOLUTE, value)

    def diveRelative(self, value):
        self.setDive(control.STATE_RELATIVE, value)

    def diveError(self, error):
        self.setDive(control.STATE_ERROR, error)

    # ---------------Forward Methods----------------

    # Forward state-value pair setter
    def setForward(self, state, value):
        self._control_msg.forward_state = state
        self._control_msg.forward = value

    def forwardAbsolute(self, value):
        self.setForward(control.STATE_ABSOLUTE, value)

    def forwardRelative(self, value):
        self.setForward(control.STATE_RELATIVE, value)

    def forwardError(self, error):
        self.setForward(control.STATE_ERROR, error)

    # ---------------Strafe Methods-----------------

    # Strafe state-value pair setter
    def setStrafeLeft(self, state, value):
        self._control_msg.strafe_state = state
        self._control_msg.strafe_left = value

    def strafeAbsolute(self, value):
        self.setStrafeLeft(control.STATE_ABSOLUTE, value)

    def strafeRelative(self, value):
        self.setStrafeLeft(control.STATE_RELATIVE, value)

    def strafeError(self, error):
        self.setStrafeLeft(control.STATE_ERROR, error)



# Main for testing
if __name__ == "__main__":
    c = control_wrapper()
    print "C obj: " + str(c)
    print c.getCurrentState()
    c.rollAbsolute(50)
    c.diveAbsolute(10)
    c.forwardAbsolute(20)
    c.yawAbsolute(20)
    c.pitchAbsolute(45)
    c.strafeAbsolute(145)
    d = control_wrapper()
    print "D obj: " + str(d)
    print d.getCurrentState()
    d.clearState()
    print c.getCurrentState()
    c.rollRelative(50)
    c.diveRelative(10)
    c.forwardRelative(20)
    c.yawRelative(20)
    c.pitchRelative(45)
    c.strafeRelative(145)
    print d.getCurrentState()
    d.clearState()
    print c.getCurrentState()
    c.rollError(50)
    c.diveError(10)
    c.forwardError(20)
    c.yawError(20)
    c.pitchError(45)
    c.strafeError(145)
    print d.getCurrentState()
