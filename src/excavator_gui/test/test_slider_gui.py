"""Tests for the slider GUI's URDF parsing and slider mapping."""

import math
import os

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')  # no display needed

from python_qt_binding.QtWidgets import QApplication  # noqa: E402

from excavator_gui.slider_gui import SLIDER_STEPS, JointSlider, parse_joints  # noqa: E402

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
    <limit lower="0.1" upper="0.4" effort="1" velocity="1"/>
  </joint>
  <joint name="nolimit_j" type="revolute"/>
</robot>
"""


@pytest.fixture(scope='module')
def qapp():
    return QApplication.instance() or QApplication([])


# ---------------------------------------------------------------- parse_joints

def test_parse_joints_skips_non_movable_joints():
    names = [name for name, _, _ in parse_joints(URDF)]
    assert names == ['rev_j', 'cont_j', 'prism_j', 'nolimit_j']


def test_parse_joints_limits():
    joints = {name: (lo, hi) for name, lo, hi in parse_joints(URDF)}
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
