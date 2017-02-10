import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QPushButton
from python_qt_binding.QtCore import QTimer

from robosub.msg import thruster
from robosub.msg import Float32Stamped
from robosub.msg import QuaternionStampedAccuracy

class SysCheck(Plugin):

    def __init__(self, context):
        super(SysCheck, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SysCheck')

        self.d = 0
        self.i = 0

        self.names = rospy.get_param('thrusters/mapping')
        self.pub = rospy.Publisher('thruster', thruster, queue_size=1)
        rospy.Subscriber('depth', Float32Stamped, self.depthSubCallback,
                         queue_size=1)
        rospy.Subscriber('orientation', QuaternionStampedAccuracy,
                         self.imuSubCallback, queue_size=1)
        self.thrusterMessage = thruster()
        rospy.Timer(rospy.Duration(1), self.sendMessage)

        self.depthTimer = QTimer(self)
        self.imuTimer = QTimer(self)

        self.depthTimer.timeout.connect(self.depthMissed)
        self.imuTimer.timeout.connect(self.imuMissed)

        self.depthTimer.start(1000)
        self.imuTimer.start(1000)

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of
        # this package
        ui_file = os.path.join(rospkg.RosPack().get_path('robosub'),
                               'src/rqt/rqt_syscheck/resource', 'SysCheck.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('SysCheckUi')

        self._widget.thrusterSpeed.valueChanged[int].connect(self.updateSpeed)

        self._widget.depthLabel.setStyleSheet("border: 5px solid green;")
        self._widget.imuLabel.setStyleSheet("border: 5px solid green;")

        # Load in the thruster buttons and connect callbacks
        self.thrusterButtons = []
        self.thrusterCallbacks = {}
        for i in range(0, len(self.names)):
            self.thrusterButtons.append(QPushButton(self.names[i]['name']))
            self.thrusterButtons[i].setCheckable(True)
            self.thrusterCallbacks[self.names[i]['name']] = \
                getattr(self, '_handle_thruster' + str(i))

            self.thrusterButtons[i].toggled[bool].connect(
                    self.thrusterCallbacks[self.names[i]['name']])
            self._widget.thrusterButtons.addWidget(self.thrusterButtons[i])
            self.thrusterMessage.data.append(0.0)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    def imuMissed(self):
        self._widget.imuLabel.setStyleSheet("border:5px solid red;")

    def imuSubCallback(self, m):
        if self.i > 5:
            self.imuTimer.stop()
            self._widget.imuLabel.setStyleSheet("border: 5px solid green;")
            self._widget.imuData.clear()
            self._widget.imuData.insertPlainText("{}".format(m))
            self.i = 0
            self.imuTimer.start(1000)
        self.i = self.i + 1

    def depthMissed(self):
        self._widget.depthLabel.setStyleSheet("border:5px solid red;")

    def depthSubCallback(self, m):
        # if self.d > 5:
        self.depthTimer.stop()
        self._widget.depthLabel.setStyleSheet("border: 5px solid green;")
        self._widget.depthData.clear()
        self._widget.depthData.insertPlainText("{}\n".format(m))
        # self.d = 0
        self.depthTimer.start(1000)
        # self.d = self.d + 1

    def sendMessage(self, e):
        self.pub.publish(self.thrusterMessage)

    def _handle_thruster0(self, state):
        if state:
            self.thrusterMessage.data[0] = 0.01 * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[0] = 0
    def _handle_thruster1(self, state):
        if state:
            self.thrusterMessage.data[1] = 0.01 * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[1] = 0
    def _handle_thruster2(self, state):
        if state:
            self.thrusterMessage.data[2] = 0.01 * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[2] = 0
    def _handle_thruster3(self, state):
        if state:
            self.thrusterMessage.data[3] = 0.01 * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[3] = 0
    def _handle_thruster4(self, state):
        if state:
            self.thrusterMessage.data[4] = 0.01 * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[4] = 0
    def _handle_thruster5(self, state):
        if state:
            self.thrusterMessage.data[5] = 0.01 * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[5] = 0
    def _handle_thruster6(self, state):
        if state:
            self.thrusterMessage.data[6] = 0.01 * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[6] = 0
    def _handle_thruster7(self, state):
        if state:
            self.thrusterMessage.data[7] = 0.01 * \
                                           self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[7] = 0

    def updateSpeed(self, value):
        self._widget.speedLabel.setText("Speed ({:+.2f})".format(
                                        float(value)/100))
        for i in range(0, len(self.thrusterMessage.data)):
            if self.thrusterButtons[i].isChecked():
                self.thrusterMessage.data[i] = float(value)/100
