import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QPushButton

from robosub.msg import thruster

class SysCheck(Plugin):

    def __init__(self, context):
        super(SysCheck, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SysCheck')

        self.names = rospy.get_param('thrusters/mapping')
        self.pub = rospy.Publisher('thruster', thruster, queue_size=1)
        self.thrusterMessage = thruster()
        rospy.Timer(rospy.Duration(1), self.sendMessage)

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

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each widget title bar
        # Usually used to open a modal configuration dialog

    def sendMessage(self,event):
        self.pub.publish(self.thrusterMessage)

    def _handle_thruster0(self, state):
        if state:
            self.thrusterMessage.data[0] = 0.01 * self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[0] = 0
    def _handle_thruster1(self, state):
        if state:
            self.thrusterMessage.data[1] = 0.01 * self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[1] = 0
    def _handle_thruster2(self, state):
        if state:
            self.thrusterMessage.data[2] = 0.01 * self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[2] = 0
    def _handle_thruster3(self, state):
        if state:
            self.thrusterMessage.data[3] = 0.01 * self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[3] = 0
    def _handle_thruster4(self, state):
        if state:
            self.thrusterMessage.data[4] = 0.01 * self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[4] = 0
    def _handle_thruster5(self, state):
        if state:
            self.thrusterMessage.data[5] = 0.01 * self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[5] = 0
    def _handle_thruster6(self, state):
        if state:
            self.thrusterMessage.data[6] = 0.01 * self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[6] = 0
    def _handle_thruster7(self, state):
        if state:
            self.thrusterMessage.data[7] = 0.01 * self._widget.thrusterSpeed.value()
        else:
            self.thrusterMessage.data[7] = 0

    def updateSpeed(self, value):
        self._widget.speedLabel.setText("Speed ({:+.2f})".format(float(value)/100))
        for i in range(0, len(self.thrusterMessage.data)):
            if self.thrusterButtons[i].isChecked():
                self.thrusterMessage.data[i] = float(value)/100
