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

        # Load in the thruster buttons and connect callbacks
        self.thusterButtons = []
        self.thruserCallbacks = {}
        for i in range(0, len(self.names)):
            self.thusterButtons.append(QPushButton(self.names[i]['name']))
            self.thruserCallbacks[self.names[i]['name']] = \
                getattr(self, '_handle_button' + str(i))

            self.thusterButtons[i].clicked[bool].connect(
                    self.thruserCallbacks[self.names[i]['name']])
            self._widget.thrusterButtons.addWidget(self.thusterButtons[i])

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

    def _handle_button0(self):
        print "Button0 pushed"
    def _handle_button1(self):
        print "Button1 pushed"
    def _handle_button2(self):
        print "Button2 pushed"
    def _handle_button3(self):
        print "Button3 pushed"
    def _handle_button4(self):
        print "Button4 pushed"
    def _handle_button5(self):
        print "Button5 pushed"
    def _handle_button6(self):
        print "Button6 pushed"
    def _handle_button7(self):
        print "Button7 pushed"
