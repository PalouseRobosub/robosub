import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QWidget

from robosub.msg import control, control_status

state_types = {
    0: "NONE",
    1: "ABSOLUTE",
    2: "RELATIVE",
    3: "ERROR"
}

class Control(Plugin):

    def __init__(self, context):
        super(Control, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Control')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of
        # this package
        ui_file = os.path.join(rospkg.RosPack().get_path('robosub'),
                               'src/rqt/rqt_control/resource', 'Control.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.control_missed)
        self.control_timer.start(1000)

        self.control_status_timer = QTimer(self)
        self.control_status_timer.timeout.connect(self.control_status_missed)
        self.control_status_timer.start(1000)

        # Give QObjects reasonable names
        self._widget.setObjectName('Control')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.statusActive.hide()
        self._widget.controlActive.hide()

        self.con_sub = rospy.Subscriber('control', control,
                                        self.control_callback, queue_size=1)
        self.cs_sub = rospy.Subscriber('control_status', control_status,
                                       self.control_status_callback,
                                       queue_size=1)
        img_file = os.path.join(rospkg.RosPack().get_path('robosub'),
                                'src/rqt/resource/robosub_logo.png')

        self._widget.setStyleSheet(".QWidget {background-image: url(" +
                                   img_file +
                                   "); background-repeat: no-repeat;" +
                                   "background-position:bottom right}")


    def control_missed(self):
        if not self._widget.controlStale.isVisible():
            self._widget.controlStale.show()
            self._widget.controlActive.hide()

    def control_status_missed(self):
        if not self._widget.statusStale.isVisible():
            self._widget.statusStale.show()
            self._widget.statusActive.hide()

    def control_status_callback(self, m):
        try:
            self.control_status_timer.stop()
        except RuntimeError:
            pass

        if self._widget.statusStale.isVisible():
            self._widget.statusStale.setVisible(False)
            self._widget.statusActive.setVisible(True)

        # Set the states
        self._widget.forwardStatusState.setText(m.forward_state)
        self._widget.strafeStatusState.setText(m.strafe_left_state)
        self._widget.diveStatusState.setText(m.dive_state)
        self._widget.rollStatusState.setText(m.roll_right_state)
        self._widget.pitchStatusState.setText(m.pitch_down_state)
        self._widget.yawStatusState.setText(m.yaw_left_state)

        self._widget.forwardGoal.setText("{:.4f}".format(m.forward_goal))
        self._widget.strafeGoal.setText("{:.4f}".format(m.strafe_left_goal))
        self._widget.diveGoal.setText("{:.4f}".format(m.dive_goal))
        self._widget.rollGoal.setText("{:.4f}".format(m.roll_right_goal))
        self._widget.pitchGoal.setText("{:.4f}".format(m.pitch_down_goal))
        self._widget.yawGoal.setText("{:.4f}".format(m.yaw_left_goal))
        self.control_status_timer.start(1000)

    def control_callback(self, m):
        try:
            self.control_timer.stop()
        except RuntimeError:
            pass

        if self._widget.controlStale.isVisible():
            self._widget.controlStale.hide()
            self._widget.controlActive.show()

        # Set the states
        self._widget.forwardState.setText(state_types[m.forward_state])
        self._widget.strafeState.setText(state_types[m.strafe_state])
        self._widget.diveState.setText(state_types[m.dive_state])
        self._widget.rollState.setText(state_types[m.roll_state])
        self._widget.pitchState.setText(state_types[m.pitch_state])
        self._widget.yawState.setText(state_types[m.yaw_state])

        self._widget.forwardValue.setText("{:.4f}".format(m.forward))
        self._widget.strafeValue.setText("{:.4f}".format(m.strafe_left))
        self._widget.diveValue.setText("{:.4f}".format(m.dive))
        self._widget.rollValue.setText("{:.4f}".format(m.roll_right))
        self._widget.pitchValue.setText("{:.4f}".format(m.pitch_down))
        self._widget.yawValue.setText("{:.4f}".format(m.yaw_left))
        self.control_timer.start(1000)

    def shutdown_plugin(self):
        self.cs_sub.unregister()
        self.con_sub.unregister()
        self.control_timer.stop()
        self.control_status_timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
