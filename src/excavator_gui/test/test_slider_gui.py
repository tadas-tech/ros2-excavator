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


# ---------------------------------------------------------------- cylinder panel

def cylinder_states(**cylinders):
    from excavator_msgs.msg import CylinderState, CylinderStates
    msg = CylinderStates()
    for name, (length, lo, hi, velocity) in cylinders.items():
        msg.cylinders.append(CylinderState(
            name=name, length=length, min_length=lo, max_length=hi,
            stroke=length - lo, velocity=velocity))
    return msg


def test_cylinder_panel_hidden_until_first_message(gui):
    assert gui.cylinder_box.isHidden()
    gui._on_cylinder_states(cylinder_states(arm_cyl=(3.0, 2.5, 4.5, 0.0)))
    assert not gui.cylinder_box.isHidden()


def test_cylinder_rows_show_stroke(gui):
    gui._on_cylinder_states(cylinder_states(
        arm_cyl=(3.0, 2.5, 4.5, 0.25), bucket_cyl=(2.9, 2.9, 3.7, -0.1)))
    arm, bucket = gui.cylinder_rows['arm_cyl'], gui.cylinder_rows['bucket_cyl']
    assert arm.bar.value() == 25
    assert arm.value_label.text() == '3.000 m   stroke 0.500 / 2.000 m   +0.25 m/s'
    assert bucket.bar.value() == 0

    gui._on_cylinder_states(cylinder_states(arm_cyl=(4.5, 2.5, 4.5, 0.0)))
    assert arm.bar.value() == 100
    assert len(gui.cylinder_rows) == 2  # rows are reused, not added again


def test_cylinder_with_unknown_range(gui):
    nan = float('nan')
    gui._on_cylinder_states(cylinder_states(odd_cyl=(1.5, nan, nan, 0.0)))
    row = gui.cylinder_rows['odd_cyl']
    assert row.bar.format() == 'range unknown'
    assert 'stroke ?' in row.value_label.text()


# ---------------------------------------------------------------- bucket-tip panel

@pytest.fixture
def tip_gui(qapp):
    from test_kinematics import URDF as ARM_URDF
    rclpy.init()
    node = Node('test_slider_gui_tip')
    gui = SliderGui(node)
    gui._on_description(String(data=ARM_URDF))
    yield gui
    gui.stop_motion()
    gui.deleteLater()
    node.destroy_node()
    rclpy.shutdown()


def shown_tip(gui):
    rows = gui.tip_rows
    return rows['reach'].value(), rows['height'].value(), rows['angle'].value()


def test_no_tip_panel_without_bucket_tip(gui):
    assert gui.arm is None
    assert gui.tip_rows == {}


def test_tip_panel_shows_the_tip(tip_gui):
    tip_gui.sliders[1].set_position(0.4)  # boom
    tip_gui._refresh_tip()
    reach, height, angle = tip_gui.arm.forward(tip_gui.joint_positions())
    assert shown_tip(tip_gui) == pytest.approx((reach, height, angle), abs=0.01)


def test_editing_height_moves_the_tip_straight_down(tip_gui):
    for js, value in zip(tip_gui.sliders, (0.0, 0.3, -1.4, -0.6)):
        js.set_position(value)
    tip_gui._refresh_tip(force=True)
    reach, height, angle = shown_tip(tip_gui)

    tip_gui.tip_rows['height'].spin.setValue(height - 0.5)  # as if typed + Enter

    assert tip_gui.arm.forward(tip_gui.joint_positions()) == pytest.approx(
        (reach, height - 0.5, angle), abs=0.01)  # display rounding: 0.01 m, 0.5 deg
    assert tip_gui.tip_status.text() == ''


def test_dragging_the_tip_slider_moves_the_joints(tip_gui):
    tip_gui._refresh_tip(force=True)
    before = tip_gui.joint_positions()
    row = tip_gui.tip_rows['reach']
    row.slider.setValue(row.slider.value() - 100)  # 1 m closer
    assert tip_gui.joint_positions() != before
    assert tip_gui.arm.forward(tip_gui.joint_positions())[0] == pytest.approx(
        row.value(), abs=1e-6)


def test_unreachable_tip_leaves_joints_alone(tip_gui):
    tip_gui._refresh_tip(force=True)
    before = tip_gui.joint_positions()
    assert not tip_gui.move_tip(20.0, 2.0, 0.0)
    assert tip_gui.joint_positions() == before
    assert 'Out of reach' in tip_gui.tip_status.text()

    tip_gui.tip_rows['reach'].spin.setValue(12.9)  # too far: display snaps back
    assert tip_gui.tip_rows['reach'].value() == pytest.approx(
        tip_gui.arm.forward(before)[0], abs=0.01)


def test_tip_edit_stops_a_preset_motion(tip_gui):
    tip_gui.move_to({'boom_joint': 1.0}, now=0.0)
    tip_gui._step_motion(now=0.5)
    reach, height, angle = tip_gui.arm.forward(tip_gui.joint_positions())
    assert tip_gui.move_tip(reach - 0.5, height, angle)
    assert tip_gui.motion is None
