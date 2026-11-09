"""Tests for the hydraulic cylinder TF publisher."""

import math

from excavator_hydraulics.cylinder_tf import (
    CylinderTf, parse_cylinders, quat_to_matrix, quat_x_to,
)
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from std_msgs.msg import String

URDF = """<?xml version="1.0"?>
<robot name="test">
  <link name="base"/>
  <link name="tip"/>
  <link name="lift_barrel"/>
  <link name="lift_rod"/>
  <link name="lonely_barrel"/>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="tip"/>
    <origin xyz="2 0 1"/>
    <axis xyz="0 -1 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="lift_barrel_joint" type="floating">
    <parent link="base"/><child link="lift_barrel"/>
    <origin xyz="0.5 0.2 0.3"/>
  </joint>
  <joint name="lift_rod_joint" type="floating">
    <parent link="tip"/><child link="lift_rod"/>
    <origin xyz="-0.4 0.2 0.6"/>
  </joint>
  <joint name="lonely_barrel_joint" type="floating">
    <parent link="base"/><child link="lonely_barrel"/>
  </joint>
</robot>
"""

BASE_PIN = np.array([0.5, 0.2, 0.3])
TIP_PIN = np.array([-0.4, 0.2, 0.6])
HINGE_OFFSET = np.array([2.0, 0.0, 1.0])


def rotate(q, v):
    return quat_to_matrix(*q) @ v


def unit(v):
    return np.asarray(v, dtype=float) / np.linalg.norm(v)


# ---------------------------------------------------------------- parsing

def test_parse_cylinders_pairs_barrel_and_rod():
    cylinders = parse_cylinders(URDF)
    assert [c.name for c in cylinders] == ['lift']  # lonely_barrel has no rod
    cyl = cylinders[0]
    assert (cyl.base, cyl.tip) == ('base', 'tip')
    np.testing.assert_allclose(cyl.base_pin, BASE_PIN)
    np.testing.assert_allclose(cyl.tip_pin, TIP_PIN)


def test_parse_cylinders_missing_origin_defaults_to_zero():
    urdf = URDF.replace('<origin xyz="0.5 0.2 0.3"/>', '')
    np.testing.assert_allclose(parse_cylinders(urdf)[0].base_pin, [0, 0, 0])


def test_parse_cylinders_none():
    assert parse_cylinders('<robot name="r"><link name="a"/></robot>') == []


# ---------------------------------------------------------------- quaternions

@pytest.mark.parametrize('q', [
    (0, 0, 0, 1),
    (0, 0, math.sin(0.5), math.cos(0.5)),
    tuple(unit([0.3, -0.5, 0.7, 0.4])),
])
def test_quat_to_matrix_is_rotation(q):
    R = quat_to_matrix(*q)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_quat_to_matrix_about_z():
    a = 0.7
    R = quat_to_matrix(0, 0, math.sin(a / 2), math.cos(a / 2))
    np.testing.assert_allclose(R @ [1, 0, 0], [math.cos(a), math.sin(a), 0], atol=1e-12)


@pytest.mark.parametrize('v', [
    [1, 0, 0],          # already aligned
    [-1, 0, 0],         # opposite: axis is undefined
    [-1, 1e-12, 0],     # nearly opposite
    [0, 1, 0],
    [0, 0, -1],
    [3, -2, 5],
    [-0.2, 0.1, -4],
])
def test_quat_x_to_rotates_x_onto_v(v):
    q = quat_x_to(np.array(v, dtype=float))
    assert np.linalg.norm(q) == pytest.approx(1.0)
    np.testing.assert_allclose(rotate(q, [1, 0, 0]), unit(v), atol=1e-9)


def test_quat_x_to_ignores_length():
    np.testing.assert_allclose(quat_x_to(np.array([2.0, 4.0, 0.0])),
                               quat_x_to(np.array([0.5, 1.0, 0.0])))


# ---------------------------------------------------------------- node

class FakeBroadcaster:

    def __init__(self):
        self.sent = []

    def sendTransform(self, transforms):
        self.sent.append(list(transforms))


@pytest.fixture
def node():
    rclpy.init()
    node = CylinderTf()
    node.tf_broadcaster = FakeBroadcaster()
    node._on_description(String(data=URDF))
    yield node
    node.destroy_node()
    rclpy.shutdown()


def set_hinge(node, angle, sec):
    """Put base -> tip into the node's TF buffer for a hinge angle (axis -Y)."""
    t = TransformStamped()
    t.header.frame_id = 'base'
    t.header.stamp.sec = sec
    t.child_frame_id = 'tip'
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
        HINGE_OFFSET
    # rotation by `angle` about -Y
    t.transform.rotation.y = -math.sin(angle / 2)
    t.transform.rotation.w = math.cos(angle / 2)
    node.tf_buffer.set_transform(t, 'test')
    return quat_to_matrix(0, -math.sin(angle / 2), 0, math.cos(angle / 2))


def by_child(transforms):
    return {t.child_frame_id: t for t in transforms}


def as_arrays(t):
    tr, r = t.transform.translation, t.transform.rotation
    return np.array([tr.x, tr.y, tr.z]), (r.x, r.y, r.z, r.w)


def test_node_finds_cylinders(node):
    assert [c.name for c in node.cylinders] == ['lift']


def test_nothing_published_without_tf(node):
    node._publish()
    assert node.tf_broadcaster.sent == []


@pytest.mark.parametrize('angle', [-1.0, 0.0, 0.4, 1.0])
def test_barrel_and_rod_point_at_each_other(node, angle):
    R = set_hinge(node, angle, sec=1)
    node._publish()

    assert len(node.tf_broadcaster.sent) == 1
    tfs = by_child(node.tf_broadcaster.sent[0])
    assert tfs.keys() == {'lift_barrel', 'lift_rod'}

    barrel = tfs['lift_barrel']
    assert barrel.header.frame_id == 'base'
    assert barrel.header.stamp.sec == 1
    pos, q = as_arrays(barrel)
    np.testing.assert_allclose(pos, BASE_PIN)
    tip_pin_in_base = R @ TIP_PIN + HINGE_OFFSET
    np.testing.assert_allclose(rotate(q, [1, 0, 0]), unit(tip_pin_in_base - BASE_PIN),
                               atol=1e-9)

    rod = tfs['lift_rod']
    assert rod.header.frame_id == 'tip'
    pos, q = as_arrays(rod)
    np.testing.assert_allclose(pos, TIP_PIN)
    base_pin_in_tip = R.T @ (BASE_PIN - HINGE_OFFSET)
    np.testing.assert_allclose(rotate(q, [1, 0, 0]), unit(base_pin_in_tip - TIP_PIN),
                               atol=1e-9)


def test_barrel_and_rod_axes_are_antiparallel_in_base_frame(node):
    R = set_hinge(node, 0.7, sec=1)
    node._publish()
    tfs = by_child(node.tf_broadcaster.sent[0])
    _, qb = as_arrays(tfs['lift_barrel'])
    _, qr = as_arrays(tfs['lift_rod'])
    barrel_x = rotate(qb, [1, 0, 0])
    rod_x_in_base = R @ rotate(qr, [1, 0, 0])
    np.testing.assert_allclose(barrel_x, -rod_x_in_base, atol=1e-9)


def test_unchanged_transform_is_not_republished(node):
    set_hinge(node, 0.2, sec=1)
    node._publish()
    node._publish()
    assert len(node.tf_broadcaster.sent) == 1

    set_hinge(node, 0.3, sec=2)
    node._publish()
    assert len(node.tf_broadcaster.sent) == 2
