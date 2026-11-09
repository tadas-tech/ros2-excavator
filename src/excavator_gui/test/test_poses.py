"""Tests for preset poses and smooth joint motion."""

import math

from excavator_gui.poses import (
    DEFAULT_VELOCITY, MIN_DURATION, Motion, PRESET_POSES, smoothstep,
)
import pytest


def test_smoothstep():
    assert smoothstep(-1) == 0.0
    assert smoothstep(0) == 0.0
    assert smoothstep(0.5) == 0.5
    assert smoothstep(1) == 1.0
    assert smoothstep(2) == 1.0


def test_presets_cover_the_digging_joints():
    for pose in PRESET_POSES.values():
        assert set(pose) == {'swing_joint', 'boom_joint', 'arm_joint', 'bucket_joint'}


def test_motion_starts_and_ends_at_the_right_positions():
    m = Motion({'a': 0.0, 'b': 1.0}, {'a': 1.0, 'b': -1.0}, {'a': 1.0, 'b': 1.0})
    assert m.positions(0.0) == {'a': 0.0, 'b': 1.0}
    assert m.positions(m.duration) == {'a': 1.0, 'b': -1.0}
    assert m.positions(m.duration * 10) == {'a': 1.0, 'b': -1.0}
    assert not m.done(m.duration * 0.99)
    assert m.done(m.duration)


def test_joints_arrive_together():
    m = Motion({'a': 0.0, 'b': 0.0}, {'a': 2.0, 'b': 0.5}, {'a': 1.0, 'b': 1.0})
    half = m.positions(m.duration / 2)
    assert half['a'] == pytest.approx(1.0)
    assert half['b'] == pytest.approx(0.25)


def test_peak_speed_stays_within_velocity_limits():
    velocities = {'a': 0.5, 'b': 2.0}
    m = Motion({'a': 0.0, 'b': 0.0}, {'a': 1.0, 'b': 3.0}, velocities)
    dt = 1e-4
    for i in range(200):
        t = m.duration * i / 200
        p0, p1 = m.positions(t), m.positions(t + dt)
        for name, v in velocities.items():
            assert abs(p1[name] - p0[name]) / dt <= v * (1 + 1e-3)


def test_slowest_joint_sets_the_duration():
    m = Motion({'a': 0.0, 'b': 0.0}, {'a': 1.0, 'b': 1.0}, {'a': 0.5, 'b': 2.0})
    assert m.duration == pytest.approx(1.5 * 1.0 / 0.5)


def test_missing_velocity_uses_default():
    m = Motion({'a': 0.0}, {'a': 2.0})
    assert m.duration == pytest.approx(1.5 * 2.0 / DEFAULT_VELOCITY)


def test_tiny_move_takes_minimum_time():
    assert Motion({'a': 0.0}, {'a': 1e-6}, {'a': 1.0}).duration == MIN_DURATION
    assert Motion({'a': 0.0}, {'a': 0.0}, {'a': 1.0}).duration == MIN_DURATION


def test_joints_not_in_target_are_left_alone():
    m = Motion({'a': 0.0, 'b': 1.0}, {'a': 1.0}, {'a': 1.0})
    assert m.positions(m.duration) == {'a': 1.0}


def test_unknown_target_joints_are_ignored():
    m = Motion({'a': 0.0}, {'a': 1.0, 'ghost': math.pi}, {'a': 1.0})
    assert set(m.target) == {'a'}
