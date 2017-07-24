# -*- coding:utf-8 -*-
# AUTHOR:   Brandon Kallaher
# FILE:     control_wrapper.py
# CREATED:  2017-07-03 23:20:06
# MODIFIED: 2017-07-11 14:03:39
# DESC:     This class is used to wrap around the control messages for
#           readability and simplicity. This uses a singleton so that all
#           instances have the same internal state at any given time and
#           therefore making relative commands possible from any instance

import rospy
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
    def __init__(self, topic="control"):
        self._control_msg = control()
        self._pub = rospy.Publisher(topic, control, queue_size=1)

    # Allow a user to get the current state
    def getCurrentState(self):
        return self._control_msg

    # Publish the message in its current state
    def publish(self):
        self._pub.publish(self._control_msg)

    # Allow for clearing of the movement state
    def clearState(self):
        self._control_msg = control()

    # ---------------Yaw Methods--------------------

    # Yaw state-value pair setter
    def setYawLeft(self, state, value):
        self._control_msg.yaw_state = state
        self._control_msg.yaw_left = value

    # Absolute Yaw method
    def yawAbsolute(self, value):
        self.setYawLeft(control.STATE_ABSOLUTE, value)

    # Relative Yaw method
    def yawRelative(self, value):
        self.setYawLeft(control.STATE_RELATIVE, value)

    # Error Yaw method
    def yawError(self, error):
        self.setYawLeft(control.STATE_ERROR, error)

    # Set the yaw state to none and clear out the value
    def yawNone(self):
        self.setYawLeft(control.STATE_NONE, 0)

    # Allow the user to get the current yaw value from the class
    def getCurrentYawValue(self):
        return self._control_msg.yaw_left

    # Allow the user to get the current yaw state from the class
    def getCurrentYawState(self):
        return self._control_msg.yaw_state

    # ---------------Pitch Methods------------------

    # Pitch state-value pair setter
    def setPitchDown(self, state, value):
        self._control_msg.pitch_state = state
        self._control_msg.pitch_down = value

    # Absolute Pitch method
    def pitchAbsolute(self, value):
        self.setPitchDown(control.STATE_ABSOLUTE, value)

    # Relative Pitch method
    def pitchRelative(self, value):
        self.setPitchDown(control.STATE_RELATIVE, value)

    # Error Pitch method
    def pitchError(self, error):
        self.setPitchDown(control.STATE_ERROR, error)

    # Set the pitch state to none and clear out the value
    def pitchNone(self):
        self.setPitchLeft(control.STATE_NONE, 0)

    # Allow the user to get the current pitch value from the class
    def getCurrentPitchValue(self):
        return self._control_msg.pitch_down

    # Allow the user to get the current pitch state from the class
    def getCurrentPitchState(self):
        return self._control_msg.pitch_state

    # ---------------Roll Methods-------------------

    # Roll state-value pair setter
    def setRollRight(self, state, value):
        self._control_msg.roll_state = state
        self._control_msg.roll_right = value

    # Absolute Roll method
    def rollAbsolute(self, value):
        self.setRollRight(control.STATE_ABSOLUTE, value)

    # Relative Roll method
    def rollRelative(self, value):
        self.setRollRight(control.STATE_RELATIVE, value)

    # Error Roll method
    def rollError(self, error):
        self.setRollRight(control.STATE_ERROR, error)

    # Set the roll state to none and clear out the value
    def rollNone(self):
        self.setRollLeft(control.STATE_NONE, 0)

    # Allow the user to get the current roll value from the class
    def getCurrentRollValue(self):
        return self._control_msg.roll_right

    # Allow the user to get the current roll state from the class
    def getCurrentRollState(self):
        return self._control_msg.roll_state

    # ---------------Dive Methods-------------------

    # Dive state-value pair setter
    def setDive(self, state, value):
        self._control_msg.dive_state = state
        self._control_msg.dive = value

    # Absolute Dive method
    def diveAbsolute(self, value):
        self.setDive(control.STATE_ABSOLUTE, value)

    # Relative Dive method
    def diveRelative(self, value):
        self.setDive(control.STATE_RELATIVE, value)

    # Error Dive method
    def diveError(self, error):
        self.setDive(control.STATE_ERROR, error)

    # Set the dive state to none and clear out the value
    def diveNone(self):
        self.setDiveLeft(control.STATE_NONE, 0)

    # Allow the user to get the current dive value from the class
    def getCurrentDiveValue(self):
        return self._control_msg.dive

    # Allow the user to get the current dive state from the class
    def getCurrentDiveState(self):
        return self._control_msg.dive_state

    # ---------------Forward Methods----------------

    # Forward state-value pair setter
    def setForward(self, state, value):
        self._control_msg.forward_state = state
        self._control_msg.forward = value

    # Absolute Forward method
    def forwardAbsolute(self, value):
        self.setForward(control.STATE_ABSOLUTE, value)

    # Relative Forward method
    def forwardRelative(self, value):
        self.setForward(control.STATE_RELATIVE, value)

    # Error Forward method
    def forwardError(self, error):
        self.setForward(control.STATE_ERROR, error)

    # Set the forward state to none and clear out the value
    def forwardNone(self):
        self.setForwardLeft(control.STATE_NONE, 0)

    # Allow the user to get the current forward value from the class
    def getCurrentForwardValue(self):
        return self._control_msg.forward

    # Allow the user to get the current forward state from the class
    def getCurrentForwardState(self):
        return self._control_msg.forward_state

    # ---------------Strafe Methods-----------------

    # Strafe state-value pair setter
    def setStrafeLeft(self, state, value):
        self._control_msg.strafe_state = state
        self._control_msg.strafe_left = value

    # Absolute Strafe method
    def strafeAbsolute(self, value):
        self.setStrafeLeft(control.STATE_ABSOLUTE, value)

    # Relative Strafe method
    def strafeRelative(self, value):
        self.setStrafeLeft(control.STATE_RELATIVE, value)

    # Error Strafe method
    def strafeError(self, error):
        self.setStrafeLeft(control.STATE_ERROR, error)

    # Set the strafe state to none and clear out the value
    def strafeNone(self):
        self.setStrafeLeft(control.STATE_NONE, 0)

    # Allow the user to get the current strafe value from the class
    def getCurrentStrafeValue(self):
        return self._control_msg.strafe_left

    # Allow the user to get the current strafe state from the class
    def getCurrentStrafeState(self):
        return self._control_msg.strafe_state



# Main for testing
if __name__ == "__main__":
    rospy.init_node("controlWrapperTest")
    c = control_wrapper("control")
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
    d.publish()
