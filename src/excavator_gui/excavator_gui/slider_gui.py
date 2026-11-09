"""
Slider GUI for the excavator.

Waits for the URDF on /robot_description, builds one slider per movable
joint (using the joint limits from the URDF) and publishes the slider
positions on /joint_states.
"""

import math
import signal
import sys
import xml.etree.ElementTree as ET

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSlider,
    QVBoxLayout, QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

SLIDER_STEPS = 1000
PUBLISH_RATE_HZ = 30.0


def parse_joints(urdf_xml):
    """Return [(name, lower, upper)] for every non-fixed joint in the URDF."""
    joints = []
    for joint in ET.fromstring(urdf_xml).iter('joint'):
        jtype = joint.get('type')
        if jtype in ('fixed', 'floating', 'planar'):
            continue
        limit = joint.find('limit')
        if jtype == 'continuous' or limit is None:
            lower, upper = -math.pi, math.pi
        else:
            lower = float(limit.get('lower', -math.pi))
            upper = float(limit.get('upper', math.pi))
        joints.append((joint.get('name'), lower, upper))
    return joints


class JointSlider:
    """A labelled slider that maps integer steps to a joint position."""

    def __init__(self, name, lower, upper):
        self.name = name
        self.lower = lower
        self.upper = upper
        self.default = min(max(0.0, lower), upper)

        self.label = QLabel(name)
        self.value_label = QLabel()
        self.value_label.setMinimumWidth(150)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.setMinimumWidth(300)
        self.slider.valueChanged.connect(self._on_slider_moved)
        self._position = self.default
        self.reset()

    def position(self):
        return self._position

    def set_position(self, value):
        # Keep the exact value; the slider handle is only an approximation.
        self._position = min(max(value, self.lower), self.upper)
        frac = (self._position - self.lower) / (self.upper - self.lower)
        self.slider.blockSignals(True)
        self.slider.setValue(round(frac * SLIDER_STEPS))
        self.slider.blockSignals(False)
        self._update_label()

    def reset(self):
        self.set_position(self.default)

    def _on_slider_moved(self, step):
        self._position = self.lower + step / SLIDER_STEPS * (self.upper - self.lower)
        self._update_label()

    def _update_label(self):
        pos = self._position
        self.value_label.setText(f'{pos:+.3f} rad  ({math.degrees(pos):+.1f}°)')


class SliderGui(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.sliders = []
        self.setWindowTitle('Excavator Joint Control')

        self.layout_main = QVBoxLayout(self)
        self.status = QLabel('Waiting for /robot_description ...')
        self.layout_main.addWidget(self.status)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.desc_sub = node.create_subscription(
            String, 'robot_description', self._on_description, qos)
        self.pub = node.create_publisher(JointState, 'joint_states', 10)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._publish)
        self.publish_timer.start(int(1000 / PUBLISH_RATE_HZ))

    def _on_description(self, msg):
        if self.sliders:
            return
        try:
            joints = parse_joints(msg.data)
        except ET.ParseError as e:
            self.status.setText(f'Failed to parse URDF: {e}')
            return

        grid = QGridLayout()
        for row, (name, lower, upper) in enumerate(joints):
            js = JointSlider(name, lower, upper)
            grid.addWidget(js.label, row, 0)
            grid.addWidget(js.slider, row, 1)
            grid.addWidget(js.value_label, row, 2)
            self.sliders.append(js)
        self.layout_main.addLayout(grid)

        buttons = QHBoxLayout()
        reset_btn = QPushButton('Reset')
        reset_btn.clicked.connect(self._reset)
        buttons.addStretch()
        buttons.addWidget(reset_btn)
        self.layout_main.addLayout(buttons)

        self.status.setText(f'Controlling {len(self.sliders)} joints')
        self.node.get_logger().info(
            'Joints: ' + ', '.join(js.name for js in self.sliders))

    def _reset(self):
        for js in self.sliders:
            js.reset()

    def _publish(self):
        if not self.sliders:
            return
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [js.name for js in self.sliders]
        msg.position = [js.position() for js in self.sliders]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Node('excavator_slider_gui')

    app = QApplication(sys.argv)
    gui = SliderGui(node)
    gui.show()

    # Process ROS callbacks from the Qt event loop.
    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    spin_timer.start(10)

    # Let Ctrl+C close the window.
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    exit_code = app.exec()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
