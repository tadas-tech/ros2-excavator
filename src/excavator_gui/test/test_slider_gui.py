"""Tests for the slider GUI's URDF parsing and slider mapping."""

import math
import os

from excavator_gui.poses import PRESET_POSES
from excavator_gui.slider_gui import JointSlider, parse_joints, SLIDER_STEPS, SliderGui
import pytest
from python_qt_binding.QtWidgets import QApplication
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

URDF = """<?xml version="1.0"?>
<robot name="test">
  <joint name="fixed_j" type="fixed"/>
  <joint name="float_j" type="floating"/>
  <joint name="planar_j" type="planar"/>
  <joint name="rev_j" type="revolute">
    <limit lower="-0.5" upper="1.5" effort="1" velocity="1"/>
  </joint>
  <joint name="cont_j" type="continuous">
    <limit effort="1" velocity="1"/>
  </joint>
  <joint name="prism_j" type="prismatic">
    <limit lower="0.1" upper="0.4" effort="1" velocity="0.5"/>
  </joint>
  <joint name="nolimit_j" type="revolute"/>
</robot>
"""


@pytest.fixture(scope='module')
def qapp():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')  # no display needed
    return QApplication.instance() or QApplication([])


# ---------------------------------------------------------------- parse_joints

def test_parse_joints_skips_non_movable_joints():
    names = [j.name for j in parse_joints(URDF)]
    assert names == ['rev_j', 'cont_j', 'prism_j', 'nolimit_j']


def test_parse_joints_limits():
    joints = {j.name: (j.lower, j.upper) for j in parse_joints(URDF)}
    assert joints['rev_j'] == (-0.5, 1.5)
    assert joints['prism_j'] == (0.1, 0.4)
    assert joints['cont_j'] == (-math.pi, math.pi)
    assert joints['nolimit_j'] == (-math.pi, math.pi)


# ---------------------------------------------------------------- JointSlider

def test_default_is_zero_when_in_range(qapp):
    js = JointSlider('j', -0.5, 1.5)
    assert js.position() == 0.0
    assert js.slider.value() == round(0.5 / 2.0 * SLIDER_STEPS)


def test_default_is_clamped_into_range(qapp):
    assert JointSlider('j', 0.1, 0.4).position() == 0.1
    assert JointSlider('j', -2.0, -1.0).position() == -1.0


def test_slider_ends_map_to_limits(qapp):
    js = JointSlider('j', -0.6, 1.1)
    js.slider.setValue(0)
    assert js.position() == pytest.approx(-0.6)
    js.slider.setValue(SLIDER_STEPS)
    assert js.position() == pytest.approx(1.1)
    js.slider.setValue(SLIDER_STEPS // 2)
    assert js.position() == pytest.approx(0.25)


def test_set_position_keeps_exact_value_and_clamps(qapp):
    js = JointSlider('j', -1.0, 1.0)
    js.set_position(0.12345)
    assert js.position() == 0.12345  # not rounded to a slider step
    js.set_position(5.0)
    assert js.position() == 1.0
    assert js.slider.value() == SLIDER_STEPS
    js.set_position(-5.0)
    assert js.position() == -1.0
    assert js.slider.value() == 0


def test_reset(qapp):
    js = JointSlider('j', -1.0, 1.0)
    js.slider.setValue(SLIDER_STEPS)
    js.reset()
    assert js.position() == 0.0


def test_label_shows_radians_and_degrees(qapp):
    js = JointSlider('j', -4.0, 4.0)
    js.set_position(math.pi / 2)
    assert js.value_label.text() == '+1.571 rad  (+90.0°)'


def test_parse_joints_velocity():
    joints = {j.name: j.velocity for j in parse_joints(URDF)}
    assert joints['rev_j'] == 1.0
    assert joints['prism_j'] == 0.5
    assert joints['nolimit_j'] is None


# ---------------------------------------------------------------- SliderGui

EXCAVATOR_URDF = """<?xml version="1.0"?>
<robot name="excavator">
  <joint name="swing_joint" type="continuous">
    <limit effort="1" velocity="1.0"/>
  </joint>
  <joint name="boom_joint" type="revolute">
    <limit lower="-0.6" upper="1.1" effort="1" velocity="0.5"/>
  </joint>
  <joint name="arm_joint" type="revolute">
    <limit lower="-2.6" upper="0.3" effort="1" velocity="0.6"/>
  </joint>
  <joint name="bucket_joint" type="revolute">
    <limit lower="-2.2" upper="1.0" effort="1" velocity="0.8"/>
  </joint>
</robot>
"""


@pytest.fixture
def gui(qapp):
    rclpy.init()
    node = Node('test_slider_gui')
    gui = SliderGui(node)
    gui._on_description(String(data=EXCAVATOR_URDF))
    yield gui
    gui.stop_motion()
    gui.deleteLater()
    node.destroy_node()
    rclpy.shutdown()


def positions(gui):
    return {js.name: js.position() for js in gui.sliders}


def test_gui_has_a_button_per_preset(gui):
    assert list(gui.pose_buttons) == list(PRESET_POSES)


@pytest.mark.parametrize('name', list(PRESET_POSES))
def test_preset_motion_reaches_pose(gui, name):
    gui.move_to(PRESET_POSES[name], now=0.0)
    duration = gui.motion.duration

    gui._step_motion(now=duration / 2)  # part way: between start and target
    for joint, target in PRESET_POSES[name].items():
        assert min(0.0, target) <= positions(gui)[joint] <= max(0.0, target)

    gui._step_motion(now=duration)
    assert gui.motion is None
    assert positions(gui) == pytest.approx(PRESET_POSES[name])


def test_preset_button_starts_motion(gui):
    gui.pose_buttons['Dig'].click()
    assert gui.motion is not None
    assert gui.motion.target == PRESET_POSES['Dig']


def test_dragging_a_slider_stops_the_motion(gui):
    gui.move_to(PRESET_POSES['Dump'], now=0.0)
    gui._step_motion(now=0.5)
    gui.sliders[1].slider.setValue(0)  # user drags boom to its lower limit
    assert gui.motion is None
    gui._step_motion(now=100.0)
    assert positions(gui)['boom_joint'] == pytest.approx(-0.6)


def test_reset_moves_back_to_defaults(gui):
    gui.move_to(PRESET_POSES['Dump'], now=0.0)
    gui._step_motion(now=100.0)
    gui._reset()
    gui._step_motion(now=time_after(gui))
    assert positions(gui) == pytest.approx({js.name: 0.0 for js in gui.sliders})


def test_targets_are_clamped_and_unknown_joints_ignored(gui):
    gui.move_to({'boom_joint': 5.0, 'no_such_joint': 1.0}, now=0.0)
    gui._step_motion(now=100.0)
    assert positions(gui)['boom_joint'] == 1.1


def time_after(gui):
    return gui.motion_start + gui.motion.duration
