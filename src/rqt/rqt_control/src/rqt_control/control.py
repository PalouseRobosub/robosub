import os
import rospy
import rospkg
from rostopic import ROSTopicHz

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QWidget
from rqt_topic.topic_info import TopicInfo

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

        # Give QObjects reasonable names
        self._widget.setObjectName('Control')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        rospy.Subscriber('control', control, self.control_callback, queue_size=1)
        rospy.Subscriber('control_status', control_status, self.control_status_callback, queue_size=1)

        self.controlInfo = TopicInfo('control', 'robosub/control')
        self.controlInfo.start_monitoring()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.control_missed)
        self._timer.start(1000)

    def control_missed(self):
        pass

    def control_status_callback(self, m):
        # Set the states
        self._widget.forwardStatusState.setText(m.forward_state)
        self._widget.strafeStatusState.setText(m.strafe_left_state)
        self._widget.diveStatusState.setText(m.dive_state)
        self._widget.rollStatusState.setText(m.roll_right_state)
        self._widget.pitchStatusState.setText(m.pitch_down_state)
        self._widget.yawStatusState.setText(m.yaw_left_state)

        self._widget.forwardGoal.setText(str(m.forward_goal))
        self._widget.strafeGoal.setText(str(m.strafe_left_goal))
        self._widget.diveGoal.setText(str(m.dive_goal))
        self._widget.rollGoal.setText(str(m.roll_right_goal))
        self._widget.pitchGoal.setText(str(m.pitch_down_goal))
        self._widget.yawGoal.setText(str(m.yaw_left_goal))

    def control_callback(self, m):
        # Set the states
        self._widget.forwardState.setText(state_types[m.forward_state])
        self._widget.strafeState.setText(state_types[m.strafe_state])
        self._widget.diveState.setText(state_types[m.dive_state])
        self._widget.rollState.setText(state_types[m.roll_state])
        self._widget.pitchState.setText(state_types[m.pitch_state])
        self._widget.yawState.setText(state_types[m.yaw_state])

        self._widget.forwardValue.setText(str(m.forward))
        self._widget.strafeValue.setText(str(m.strafe_left))
        self._widget.diveValue.setText(str(m.dive))
        self._widget.rollValue.setText(str(m.roll_right))
        self._widget.pitchValue.setText(str(m.pitch_down))
        self._widget.yawValue.setText(str(m.yaw_left))

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

