"""Tests for bucket-tip forward and inverse kinematics."""

import math
import random

from excavator_gui.kinematics import PlanarArm
import pytest

# Boom pivot 1 m forward and 2 m up; boom 4 m, arm 2 m; tip 1 m along the bucket.
URDF = """<?xml version="1.0"?>
<robot name="test">
  <link name="root"/><link name="house"/><link name="boom"/><link name="arm"/>
  <link name="bucket"/><link name="bucket_tip"/>
  <joint name="swing_joint" type="continuous">
    <parent link="root"/><child link="house"/>
    <origin xyz="0 0 1.5"/><axis xyz="0 0 1"/>
    <limit effort="1" velocity="1"/>
  </joint>
  <joint name="boom_joint" type="revolute">
    <parent link="house"/><child link="boom"/>
    <origin xyz="1 -0.3 0.5"/><axis xyz="0 -1 0"/>
    <limit lower="-1" upper="1.5" effort="1" velocity="1"/>
  </joint>
  <joint name="arm_joint" type="revolute">
    <parent link="boom"/><child link="arm"/>
    <origin xyz="4 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-2.5" upper="0.5" effort="1" velocity="1"/>
  </joint>
  <joint name="bucket_joint" type="revolute">
    <parent link="arm"/><child link="bucket"/>
    <origin xyz="2 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="-2" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="bucket_tip_joint" type="fixed">
    <parent link="bucket"/><child link="bucket_tip"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>
"""


@pytest.fixture
def arm():
    return PlanarArm.from_urdf(URDF)


def random_pose(arm, rng):
    return {j.name: rng.uniform(j.lower, j.upper) for j in arm.joints}


# ---------------------------------------------------------------- building

def test_from_urdf(arm):
    assert [j.name for j in arm.joints] == ['boom_joint', 'arm_joint', 'bucket_joint']
    assert arm.joints[0].offset == (1.0, 2.0)  # boom pivot: house 1.5 up, + (1, 0.5)
    assert arm.tip == (1.0, 0.0)


@pytest.mark.parametrize('change, message', [
    (('<link name="bucket_tip"/>', ''), 'bucket_tip'),
    (('name="arm_joint"', 'name="stick_joint"'), 'arm_joint'),
    (('<axis xyz="0 -1 0"/>\n    <limit lower="-2.5"',
      '<axis xyz="0 0 1"/>\n    <limit lower="-2.5"'), 'pitch'),
    (('<parent link="arm"/><child link="bucket"/>', '<parent link="boom"/><child link="bucket"/>'),
     'does not follow'),
])
def test_from_urdf_rejects_other_shapes(change, message):
    urdf = URDF.replace(*change)
    if message == 'bucket_tip':
        urdf = urdf.replace('<child link="bucket_tip"/>', '<child link="elsewhere"/>')
    with pytest.raises(ValueError, match=message):
        PlanarArm.from_urdf(urdf)


# ---------------------------------------------------------------- forward

def test_forward_all_zero(arm):
    reach, height, angle = arm.forward({})
    assert (reach, height, angle) == pytest.approx((1 + 4 + 2 + 1, 2.0, 0.0))


def test_forward_boom_raised_90_degrees(arm):
    reach, height, angle = arm.forward({'boom_joint': math.pi / 2})
    assert (reach, height, angle) == pytest.approx((1.0, 2 + 7, math.pi / 2))


def test_forward_arm_hanging_down(arm):
    reach, height, angle = arm.forward({'arm_joint': -math.pi / 2, 'bucket_joint': -0.5})
    assert reach == pytest.approx(1 + 4 + math.cos(-math.pi / 2 - 0.5))
    assert height == pytest.approx(2 - 2 + math.sin(-math.pi / 2 - 0.5))
    assert angle == pytest.approx(-math.pi / 2 - 0.5)


def test_axis_plus_y_flips_the_sign():
    flipped = PlanarArm.from_urdf(URDF.replace(
        '<axis xyz="0 -1 0"/>\n    <limit lower="-1" upper="1.5"',
        '<axis xyz="0 1 0"/>\n    <limit lower="-1.5" upper="1"'))
    assert flipped.forward({'boom_joint': -0.4}) == pytest.approx(
        PlanarArm.from_urdf(URDF).forward({'boom_joint': 0.4}))


# ---------------------------------------------------------------- inverse

def test_inverse_round_trip(arm):
    rng = random.Random(0)
    for _ in range(500):
        q = random_pose(arm, rng)
        target = arm.forward(q)
        solution = arm.inverse(*target, current=q)
        assert solution is not None
        assert arm.forward(solution) == pytest.approx(target, abs=1e-9)
        for joint in arm.joints:
            assert joint.lower <= solution[joint.name] <= joint.upper


def test_inverse_returns_the_closest_solution(arm):
    rng = random.Random(1)
    for _ in range(200):
        q = random_pose(arm, rng)
        solution = arm.inverse(*arm.forward(q), current=q)
        assert solution == pytest.approx(q, abs=1e-9)


def test_inverse_out_of_reach(arm):
    assert arm.inverse(20.0, 2.0, 0.0) is None   # beyond the boom + arm length
    assert arm.inverse(1.0, 2.0, 0.0) is None    # tip on the boom pivot


def test_inverse_outside_joint_limits(arm):
    # Reachable distance, but only by bending the arm the wrong way (arm upper limit 0.5).
    assert arm.inverse(*arm.forward({'boom_joint': 1.0, 'arm_joint': 1.5}),
                       current={'boom_joint': 1.0}) is None


def test_inverse_wraps_the_bucket_angle(arm):
    q = {'boom_joint': 0.5, 'arm_joint': -2.0, 'bucket_joint': -1.5}
    reach, height, angle = arm.forward(q)
    solution = arm.inverse(reach, height, angle + 2 * math.pi, current=q)
    assert solution == pytest.approx(q, abs=1e-9)


def test_moving_the_tip_straight_down_keeps_reach_and_angle(arm):
    q = {'boom_joint': 0.3, 'arm_joint': -1.4, 'bucket_joint': -0.6}
    reach, height, angle = arm.forward(q)
    for dz in (0.1, 0.2, 0.3):
        q = arm.inverse(reach, height - dz, angle, current=q)
        assert arm.forward(q) == pytest.approx((reach, height - dz, angle), abs=1e-9)
