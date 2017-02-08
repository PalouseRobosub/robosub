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

    def _handle_thruster0(self, state):
        if state:
            print "thruster0(" + self.names[0]['name']  + ") pushed on"
        else:
            print "thruster0(" + self.names[0]['name'] + ") pushed off"
    def _handle_thruster1(self, state):
        if state:
            print "thruster1(" + self.names[1]['name']  + ") pushed on"
        else:
            print "thruster1(" + self.names[1]['name'] + ") pushed off"
    def _handle_thruster2(self, state):
        if state:
            print "thruster2(" + self.names[2]['name']  + ") pushed on"
        else:
            print "thruster2(" + self.names[2]['name'] + ") pushed off"
    def _handle_thruster3(self, state):
        if state:
            print "thruster3(" + self.names[3]['name']  + ") pushed on"
        else:
            print "thruster3(" + self.names[3]['name'] + ") pushed off"
    def _handle_thruster4(self, state):
        if state:
            print "thruster4(" + self.names[4]['name']  + ") pushed on"
        else:
            print "thruster4(" + self.names[4]['name'] + ") pushed off"
    def _handle_thruster5(self, state):
        if state:
            print "thruster5(" + self.names[5]['name']  + ") pushed on"
        else:
            print "thruster5(" + self.names[5]['name'] + ") pushed off"
    def _handle_thruster6(self, state):
        if state:
            print "thruster6(" + self.names[6]['name']  + ") pushed on"
        else:
            print "thruster6(" + self.names[6]['name'] + ") pushed off"
    def _handle_thruster7(self, state):
        if state:
            print "thruster7(" + self.names[7]['name']  + ") pushed on"
        else:
            print "thruster7(" + self.names[7]['name'] + ") pushed off"
