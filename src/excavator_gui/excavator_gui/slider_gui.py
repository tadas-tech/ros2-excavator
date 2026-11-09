"""
Slider GUI for the excavator.

Waits for the URDF on /robot_description, builds one slider per movable
joint (using the joint limits from the URDF) and publishes the slider
positions on /joint_states. Preset buttons move the joints smoothly to a
pose; dragging a slider stops the motion. The bucket-tip panel moves the
tip of the bucket to a reach, height and bucket angle (inverse kinematics),
and a panel shows each hydraulic cylinder's stroke from /cylinder_states.
"""

from collections import namedtuple
import math
import signal
import sys
import time
import xml.etree.ElementTree as ET

from excavator_gui.kinematics import PlanarArm
from excavator_gui.poses import Motion, PRESET_POSES
from excavator_msgs.msg import CylinderStates

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
    QProgressBar, QPushButton, QSlider, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

SLIDER_STEPS = 1000
PUBLISH_RATE_HZ = 30.0
MOTION_RATE_HZ = 50.0

Joint = namedtuple('Joint', 'name lower upper velocity')


def parse_joints(urdf_xml):
    """Return a Joint for every movable joint in the URDF (velocity None if unset)."""
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
        velocity = limit.get('velocity') if limit is not None else None
        velocity = float(velocity) if velocity and float(velocity) > 0 else None
        joints.append(Joint(joint.get('name'), lower, upper, velocity))
    return joints


class JointSlider:
    """A labelled slider that maps integer steps to a joint position."""

    def __init__(self, name, lower, upper, velocity=None, on_user_move=None):
        self.name = name
        self.lower = lower
        self.upper = upper
        self.velocity = velocity
        self.default = min(max(0.0, lower), upper)
        self.on_user_move = on_user_move

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
        if self.on_user_move:
            self.on_user_move()

    def _update_label(self):
        pos = self._position
        self.value_label.setText(f'{pos:+.3f} rad  ({math.degrees(pos):+.1f}°)')


class TipRow:
    """
    Slider plus number box for one bucket-tip coordinate.

    Values are shown in display units (`factor` converts from internal
    units, e.g. rad -> deg). Calls `on_change()` when the user edits either.
    """

    def __init__(self, name, unit, lower, upper, step, decimals, on_change, factor=1.0):
        self.factor = factor
        self.step = step
        self.on_change = on_change
        self.label = QLabel(name)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(round(lower / step), round(upper / step))
        self.slider.setMinimumWidth(300)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(lower, upper)
        self.spin.setDecimals(decimals)
        self.spin.setSingleStep(step)
        self.spin.setSuffix(f' {unit}')
        self.spin.setKeyboardTracking(False)  # apply typed values on Enter, not per key
        self.spin.setMinimumWidth(150)
        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)

    def value(self):
        """Return the current value in internal units."""
        return self.spin.value() / self.factor

    def show_value(self, value, force=False):
        """Display `value` (internal units) unless the user is editing this row."""
        if not force and (self.slider.isSliderDown() or self.spin.hasFocus()):
            return
        shown = value * self.factor
        for widget, v in ((self.spin, shown), (self.slider, round(shown / self.step))):
            widget.blockSignals(True)
            widget.setValue(v)
            widget.blockSignals(False)

    def _from_slider(self, step):
        self.spin.blockSignals(True)
        self.spin.setValue(step * self.step)
        self.spin.blockSignals(False)
        self.on_change()

    def _from_spin(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(round(value / self.step))
        self.slider.blockSignals(False)
        self.on_change()


class CylinderRow:
    """Name, stroke bar (0 % retracted .. 100 % extended) and readout for one cylinder."""

    def __init__(self, name):
        self.label = QLabel(name)
        self.bar = QProgressBar()
        self.bar.setRange(0, 100)
        self.bar.setMinimumWidth(300)
        self.value_label = QLabel()
        self.value_label.setMinimumWidth(260)

    def update(self, state):
        full = state.max_length - state.min_length
        if math.isfinite(full) and full > 0:
            percent = min(max(state.stroke / full, 0.0), 1.0) * 100
            self.bar.setValue(round(percent))
            self.bar.setFormat(f'{percent:.0f} %')
            stroke = f'stroke {state.stroke:.3f} / {full:.3f} m'
        else:
            self.bar.setValue(0)
            self.bar.setFormat('range unknown')
            stroke = 'stroke ?'
        self.value_label.setText(
            f'{state.length:.3f} m   {stroke}   {state.velocity:+.2f} m/s')


class SliderGui(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.sliders = []
        self.motion = None
        self.motion_start = 0.0
        self.arm = None       # PlanarArm, if the URDF has a bucket_tip
        self.tip_rows = {}
        self.setWindowTitle('Excavator Joint Control')

        self.layout_main = QVBoxLayout(self)
        self.status = QLabel('Waiting for /robot_description ...')
        self.layout_main.addWidget(self.status)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.desc_sub = node.create_subscription(
            String, 'robot_description', self._on_description, qos)
        self.pub = node.create_publisher(JointState, 'joint_states', 10)
        self.cylinder_rows = {}
        self.cylinder_grid = QGridLayout()
        self.cylinder_box = QGroupBox('Cylinders')
        self.cylinder_box.setLayout(self.cylinder_grid)
        self.cylinder_box.hide()  # until the first /cylinder_states message
        self.cyl_sub = node.create_subscription(
            CylinderStates, 'cylinder_states', self._on_cylinder_states, qos)

        self.publish_timer = QTimer(self)
        self.publish_timer.timeout.connect(self._publish)
        self.publish_timer.start(int(1000 / PUBLISH_RATE_HZ))

        self.motion_timer = QTimer(self)
        self.motion_timer.timeout.connect(self._step_motion)

    def _on_description(self, msg):
        if self.sliders:
            return
        try:
            joints = parse_joints(msg.data)
        except ET.ParseError as e:
            self.status.setText(f'Failed to parse URDF: {e}')
            return

        grid = QGridLayout()
        for row, joint in enumerate(joints):
            js = JointSlider(*joint, on_user_move=self.stop_motion)
            grid.addWidget(js.label, row, 0)
            grid.addWidget(js.slider, row, 1)
            grid.addWidget(js.value_label, row, 2)
            self.sliders.append(js)
        self.layout_main.addLayout(grid)

        buttons = QHBoxLayout()
        self.pose_buttons = {}
        for name, pose in PRESET_POSES.items():
            btn = QPushButton(name)
            btn.clicked.connect(lambda _=False, pose=pose: self.move_to(pose))
            buttons.addWidget(btn)
            self.pose_buttons[name] = btn
        buttons.addStretch()
        reset_btn = QPushButton('Reset')
        reset_btn.clicked.connect(self._reset)
        buttons.addWidget(reset_btn)
        self.layout_main.addLayout(buttons)
        self._build_tip_panel(msg.data)
        self.layout_main.addWidget(self.cylinder_box)

        self.status.setText(f'Controlling {len(self.sliders)} joints')
        self.node.get_logger().info(
            'Joints: ' + ', '.join(js.name for js in self.sliders))

    def _build_tip_panel(self, urdf_xml):
        try:
            self.arm = PlanarArm.from_urdf(urdf_xml)
        except (ValueError, ET.ParseError) as e:
            self.node.get_logger().info(f'No bucket-tip control: {e}')
            return
        box = QGroupBox('Bucket tip')
        grid = QGridLayout(box)
        rows = (
            ('reach', TipRow('reach', 'm', 0.0, 13.0, 0.01, 2, self._on_tip_edit)),
            ('height', TipRow('height', 'm', -6.0, 10.0, 0.01, 2, self._on_tip_edit)),
            ('angle', TipRow('bucket angle', '°', -180.0, 180.0, 0.5, 1, self._on_tip_edit,
                             factor=180.0 / math.pi)),
        )
        for r, (key, row) in enumerate(rows):
            grid.addWidget(row.label, r, 0)
            grid.addWidget(row.slider, r, 1)
            grid.addWidget(row.spin, r, 2)
            self.tip_rows[key] = row
        self.tip_status = QLabel()
        self.tip_status.setStyleSheet('color: #c0392b')
        grid.addWidget(self.tip_status, len(rows), 0, 1, 3)
        self.layout_main.addWidget(box)
        self._refresh_tip(force=True)

    def joint_positions(self):
        return {js.name: js.position() for js in self.sliders}

    def _refresh_tip(self, force=False):
        """Show the tip position for the current joint positions."""
        if self.arm is None:
            return
        reach, height, angle = self.arm.forward(self.joint_positions())
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        for key, value in (('reach', reach), ('height', height), ('angle', angle)):
            self.tip_rows[key].show_value(value, force)

    def move_tip(self, reach, height, angle):
        """Move the joints so the bucket tip is at (reach, height, angle); False if unreachable."""
        current = self.joint_positions()
        solution = self.arm.inverse(reach, height, angle, current)
        if solution is None:
            self.tip_status.setText('Out of reach: the joints cannot get the tip there.')
            return False
        self.stop_motion()
        for js in self.sliders:
            if js.name in solution:
                js.set_position(solution[js.name])
        self.tip_status.setText('')
        return True

    def _on_tip_edit(self):
        rows = self.tip_rows
        if not self.move_tip(rows['reach'].value(), rows['height'].value(), rows['angle'].value()):
            self._refresh_tip(force=True)  # put the display back where the tip is

    def _on_cylinder_states(self, msg):
        for state in msg.cylinders:
            row = self.cylinder_rows.get(state.name)
            if row is None:
                row = CylinderRow(state.name)
                r = len(self.cylinder_rows)
                self.cylinder_grid.addWidget(row.label, r, 0)
                self.cylinder_grid.addWidget(row.bar, r, 1)
                self.cylinder_grid.addWidget(row.value_label, r, 2)
                self.cylinder_rows[state.name] = row
            row.update(state)
        self.cylinder_box.show()

    def _reset(self):
        self.move_to({js.name: js.default for js in self.sliders})

    def move_to(self, pose, now=None):
        """Start a smooth motion to `pose` ({joint: position}); unknown joints are ignored."""
        by_name = {js.name: js for js in self.sliders}
        target = {name: min(max(value, by_name[name].lower), by_name[name].upper)
                  for name, value in pose.items() if name in by_name}
        self.motion = Motion(
            {js.name: js.position() for js in self.sliders}, target,
            {js.name: js.velocity for js in self.sliders})
        self.motion_start = time.monotonic() if now is None else now
        self.motion_timer.start(int(1000 / MOTION_RATE_HZ))

    def stop_motion(self):
        self.motion = None
        self.motion_timer.stop()

    def _step_motion(self, now=None):
        if self.motion is None:
            return
        elapsed = (time.monotonic() if now is None else now) - self.motion_start
        by_name = {js.name: js for js in self.sliders}
        for name, value in self.motion.positions(elapsed).items():
            by_name[name].set_position(value)
        if self.motion.done(elapsed):
            self.stop_motion()

    def _publish(self):
        if not self.sliders:
            return
        self._refresh_tip()
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
