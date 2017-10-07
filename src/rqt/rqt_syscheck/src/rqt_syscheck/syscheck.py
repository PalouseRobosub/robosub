# Ros Imports
import os
import rospy
import rospkg

# Import Qt/rQt Modules
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

# Attempt to load QWidget and QPushButton from pyqt4
try:
    from python_qt_binding.QtGui import QWidget, QPushButton
# Else load from pyqt5
except ImportError:
    from python_qt_binding.QtWidgets import QWidget, QPushButton

from python_qt_binding.QtCore import QTimer

# Import Messages
from robosub.msg import thruster
from robosub.msg import Float32Stamped
from robosub.msg import Euler

class SysCheck(Plugin):

    def __init__(self, context):
        super(SysCheck, self).__init__(context)

        # Set the name of the object
        #   (Usually should be the same as the class name)
        self.setObjectName('SysCheck')


        # Get the thruster parameters
        self.names = []
        try:
            self.names = rospy.get_param('thrusters/mapping')
        except KeyError:
            print "Thruster mapping not loaded into parameter server"

        # Setup the publisher and message object for sending thruster messages
        self.pub = rospy.Publisher('thruster', thruster, queue_size=1)
        self.thrusterMessage = thruster()

        # Subscribe to the depth and orientation topics
        self.depth_sub = rospy.Subscriber('depth', Float32Stamped,
                                          self.depthSubCallback, queue_size=1)
        self.imu_sub = rospy.Subscriber('pretty/orientation',
                                        Euler,
                                        self.imuSubCallback, queue_size=1)

        # Initialize the timers
        self.depthTimer = QTimer(self)
        self.imuTimer = QTimer(self)
        self.sendTimer = QTimer(self)

        self.depthTimer.timeout.connect(self.depthMissed)
        self.imuTimer.timeout.connect(self.imuMissed)
        self.sendTimer.timeout.connect(self.sendMessage)

        self.depthTimer.start(1000)
        self.imuTimer.start(1000)

        # Only start the param timer if the params aren't loaded
        if len(self.names) == 0:
            self.paramTimer = QTimer(self)
            self.paramTimer.timeout.connect(self.loadParam)
            self.paramTimer.start(1000)

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of
        # this package
        ui_file = os.path.join(rospkg.RosPack().get_path('robosub'),
                               'src/rqt/rqt_syscheck/resource', 'SysCheck.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects a name (Usually the class name + 'Ui')
        self._widget.setObjectName('SysCheckUi')

        # Add RoboSub Logo to the GUI
        logo_file = os.path.join(rospkg.RosPack().get_path('robosub'),
                                 'src/rqt/resource', 'robosub_logo.png')
        self._widget.setStyleSheet(".QWidget {background-image: url(" +
                                   logo_file +
                                   "); background-repeat: no-repeat;" +
                                   "background-position:bottom right}")

        # Hide the stale labels on init
        self._widget.imuStale.hide()
        self._widget.depthStale.hide()

        # Connect the valueChanged signal to our updateSpeed function
        self._widget.thrusterSpeed.valueChanged[int].connect(self.updateSpeed)

        self._widget.thrusterEnable.setCheckable(True)
        self._widget.thrusterEnable.toggled[bool].connect(self.enable)
        self._widget.thrusterKill.clicked[bool].connect(self.kill)

        # Load in the thruster buttons and connect callbacks
        self.thrusterButtons = []
        self.thrusterScales = []
        self.thrusterCallbacks = {}
        self.loadThrusters()

        # If the context is not the root add the serial number to the window
        #   title
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def kill(self):
        for i in self.thrusterButtons:
            i.setChecked(False)
        self._widget.thrusterSpeed.setValue(0)


    def enable(self, s):
        if s:
            self.sendTimer.start(1000)
        else:
            self.sendTimer.stop()

    def loadThrusters(self):
        # Loop over all of the thruster values found in the params
        for i in range(0, len(self.names)):
            # Add a button to a list so we can mess with it later
            self.thrusterButtons.append(QPushButton(self.names[i]['name']))
            # Modify setting of the button
            self.thrusterButtons[i].setCheckable(True)

            # Save the callbacks in a list
            self.thrusterCallbacks[self.names[i]['name']] = \
                getattr(self, '_handle_thruster' + str(i))

            # Connect the callback to the button's toggle event
            self.thrusterButtons[i].toggled[bool].connect(
                    self.thrusterCallbacks[self.names[i]['name']])

            # Add the button to the Ui
            self._widget.thrusterButtons.addWidget(self.thrusterButtons[i])


            # Get the orientation
            self.thrusterScales.append(0)
            for v in self.names[i]['orientation'].values():
                self.thrusterScales[i] = self.thrusterScales[i] + v

            # Append a value to the thruster message for this button
            self.thrusterMessage.data.append(0.0)
        print self.thrusterScales

    def loadParam(self):
        try:
            self.names = rospy.get_param('thrusters/mapping')
            self.loadThrusters()
            # Stop the timer if the params were successfully loaded
            self.paramTimer.stop()
        except KeyError:
            # Don't throw an error if we hit a KeyError
            pass

    def shutdown_plugin(self):
        # Stop the send timer before unregistering the publisher
        self.sendTimer.stop()
        # Unregister the thruster publisher and subscribers
        self.pub.unregister()
        self.depth_sub.unregister()
        self.imu_sub.unregister()

        # Stop the Other Timers
        try:
            self.paramTimer.stop()
        except AttributeError:
            pass
        self.imuTimer.stop()
        self.depthTimer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def imuMissed(self):
        # If an Imu message was not received by the time the timer fired
        #   show the stale label and hide the active label
        if not self._widget.imuStale.isVisible():
            self._widget.imuStale.show()
        if self._widget.imuActive.isVisible():
            self._widget.imuActive.hide()

    def imuSubCallback(self, m):
        # Stop the imuTimer so it doesn't fire in this function
        self.imuTimer.stop()

        # Reset the active label hiding the stale label
        if self._widget.imuStale.isVisible():
            self._widget.imuStale.hide()
        if not self._widget.imuActive.isVisible():
            self._widget.imuActive.show()

        self._widget.currentRoll.setText(str(m.roll))
        self._widget.currentPitch.setText(str(m.pitch))
        self._widget.currentYaw.setText(str(m.yaw))

        # Restart the timer
        self.imuTimer.start(1000)

    def depthMissed(self):
        # If an Depth message was not received by the time the timer fired
        #   show the stale label and hide the active label
        if not self._widget.depthStale.isVisible():
            self._widget.depthStale.show()
        if self._widget.depthActive.isVisible():
            self._widget.depthActive.hide()

    def depthSubCallback(self, m):
        # Stop the imuTimer so it doesn't fire in this function
        self.depthTimer.stop()

        # Reset the active label hiding the stale label
        if self._widget.depthStale.isVisible():
            self._widget.depthStale.hide()
        if not self._widget.depthActive.isVisible():
            self._widget.depthActive.show()

        self._widget.currentDepth.setText(str(m.data))

        # Restart the timer
        self.depthTimer.start(1000)

    def sendMessage(self):
        # Publish the message that we have constructed
        self.pub.publish(self.thrusterMessage)

    # Update the speeds in the thruster message based on the slider
    def updateSpeed(self, value):
        # Update the speed label so the user knows the value that is set
        self._widget.speedLabel.setText("Speed ({:+.2f})".format(
                                        float(value)/100))

        # Loop over the thruster message and update the value
        for i in range(0, len(self.thrusterMessage.data)):
            # Check if the thruster is enabled
            if self.thrusterButtons[i].isChecked():
                self.thrusterMessage.data[i] = self.thrusterScales[i] * \
                                               float(value)/100

    '''
        The following functions handle updating the thruster message based on
        the buttons.
    '''
    def _handle_thruster0(self, state):
        if state:
            self.thrusterMessage.data[0] = 0.01 * self.thrusterScales[0] * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[0] = 0

    def _handle_thruster1(self, state):
        if state:
            self.thrusterMessage.data[1] = 0.01 * self.thrusterScales[1] * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[1] = 0

    def _handle_thruster2(self, state):
        if state:
            self.thrusterMessage.data[2] = 0.01 * self.thrusterScales[2] * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[2] = 0

    def _handle_thruster3(self, state):
        if state:
            self.thrusterMessage.data[3] = 0.01 * self.thrusterScales[3] * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[3] = 0

    def _handle_thruster4(self, state):
        if state:
            self.thrusterMessage.data[4] = 0.01 * self.thrusterScales[4] * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[4] = 0

    def _handle_thruster5(self, state):
        if state:
            self.thrusterMessage.data[5] = 0.01 * self.thrusterScales[5] * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[5] = 0

    def _handle_thruster6(self, state):
        if state:
            self.thrusterMessage.data[6] = 0.01 * self.thrusterScales[6] * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[6] = 0

    def _handle_thruster7(self, state):
        if state:
            self.thrusterMessage.data[7] = 0.01 * self.thrusterScales[7] * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[7] = 0
