"""Checks on the excavator URDF produced by xacro."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import xacro

XACRO_PATH = Path(__file__).resolve().parent.parent / 'urdf' / 'excavator.urdf.xacro'

# The joint table in the README.
EXPECTED_JOINTS = {
    'swing_joint': (-math.pi, math.pi),
    'boom_joint': (-0.6, 1.1),
    'arm_joint': (-2.6, 0.3),
    'bucket_joint': (-2.2, 1.0),
}

CYLINDERS = ['boom_cyl_left', 'boom_cyl_right', 'arm_cyl', 'bucket_cyl']

SWEEP_STEPS = 2001


@pytest.fixture(scope='module')
def robot():
    return ET.fromstring(xacro.process_file(str(XACRO_PATH)).toxml())


def joints_by_child(robot):
    return {j.find('child').get('link'): j for j in robot.iter('joint')}


def vec(element, attr, default='0 0 0'):
    value = element.get(attr, default) if element is not None else default
    return np.array([float(v) for v in value.split()])


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_matrix(axis, angle):
    k = axis / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def cylinder_length(link):
    return float(link.find('visual/geometry/cylinder').get('length'))


# ---------------------------------------------------------------- structure

def test_xacro_produces_robot(robot):
    assert robot.tag == 'robot'
    assert robot.get('name') == 'excavator'


def test_links_form_a_single_tree(robot):
    links = {link.get('name') for link in robot.iter('link')}
    by_child = joints_by_child(robot)

    for joint in robot.iter('joint'):
        assert joint.find('parent').get('link') in links, joint.get('name')
        assert joint.find('child').get('link') in links, joint.get('name')

    assert len(by_child) == len(list(robot.iter('joint'))), 'a link has two parent joints'
    roots = links - by_child.keys()
    assert roots == {'base_footprint'}


def test_names_are_unique(robot):
    for tag in ('link', 'joint'):
        names = [e.get('name') for e in robot.iter(tag)]
        assert len(names) == len(set(names)), f'duplicate {tag} names'


def test_movable_joints_match_readme(robot):
    movable = {}
    for joint in robot.iter('joint'):
        if joint.get('type') in ('revolute', 'continuous', 'prismatic'):
            limit = joint.find('limit')
            movable[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))

    assert movable.keys() == EXPECTED_JOINTS.keys()
    for name, (lower, upper) in EXPECTED_JOINTS.items():
        assert movable[name] == pytest.approx((lower, upper), abs=1e-3), name


@pytest.mark.parametrize('name', ['boom_joint', 'arm_joint', 'bucket_joint'])
def test_pitch_joints_raise_with_positive_angle(robot, name):
    joint = next(j for j in robot.iter('joint') if j.get('name') == name)
    np.testing.assert_allclose(vec(joint.find('axis'), 'xyz'), [0, -1, 0])


def test_links_with_inertia_have_positive_mass(robot):
    for link in robot.iter('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        name = link.get('name')
        assert float(inertial.find('mass').get('value')) > 0, name
        inertia = inertial.find('inertia')
        ixx, iyy, izz = (float(inertia.get(k)) for k in ('ixx', 'iyy', 'izz'))
        assert min(ixx, iyy, izz) > 0, name
        # triangle inequality for principal moments
        assert ixx + iyy >= izz and iyy + izz >= ixx and ixx + izz >= iyy, name


def test_root_link_has_no_inertia(robot):
    # KDL (robot_state_publisher) does not support inertia on the root link.
    root = next(link for link in robot.iter('link') if link.get('name') == 'base_footprint')
    assert root.find('inertial') is None


# ---------------------------------------------------------------- cylinders

def test_every_cylinder_has_barrel_and_rod(robot):
    floating = {j.find('child').get('link') for j in robot.iter('joint')
                if j.get('type') == 'floating'}
    expected = {f'{c}_{part}' for c in CYLINDERS for part in ('barrel', 'rod')}
    assert floating == expected


def pin_distances(robot, name):
    """Pin-to-pin distance of a cylinder over its joint's range.

    Returns (distances, barrel_length, rod_length).
    """
    by_child = joints_by_child(robot)
    links = {link.get('name'): link for link in robot.iter('link')}

    barrel_joint = by_child[f'{name}_barrel']
    rod_joint = by_child[f'{name}_rod']
    base = barrel_joint.find('parent').get('link')
    tip = rod_joint.find('parent').get('link')
    base_pin = vec(barrel_joint.find('origin'), 'xyz')
    tip_pin = vec(rod_joint.find('origin'), 'xyz')

    # The tip link must hang directly from the base link by one revolute joint.
    joint = by_child[tip]
    assert joint.find('parent').get('link') == base, f'{name}: {tip} is not a child of {base}'
    assert joint.get('type') == 'revolute'

    origin = joint.find('origin')
    offset = vec(origin, 'xyz')
    fixed_rot = rpy_matrix(*vec(origin, 'rpy'))
    axis = vec(joint.find('axis'), 'xyz', '1 0 0')
    limit = joint.find('limit')
    angles = np.linspace(float(limit.get('lower')), float(limit.get('upper')), SWEEP_STEPS)

    distances = np.array([
        np.linalg.norm(offset + fixed_rot @ axis_angle_matrix(axis, q) @ tip_pin - base_pin)
        for q in angles
    ])
    return (distances, cylinder_length(links[f'{name}_barrel']),
            cylinder_length(links[f'{name}_rod']))


@pytest.mark.parametrize('name', CYLINDERS)
def test_cylinder_fits_over_joint_range(robot, name):
    """Barrel and rod each fit the shortest pin distance; together they reach the longest."""
    distances, barrel, rod = pin_distances(robot, name)
    shortest, longest = distances.min(), distances.max()

    assert barrel <= shortest, (
        f'{name}: barrel {barrel:.3f} m longer than shortest pin distance {shortest:.3f} m')
    assert rod <= shortest, (
        f'{name}: rod {rod:.3f} m longer than shortest pin distance {shortest:.3f} m')
    assert barrel + rod >= longest, (
        f'{name}: barrel + rod {barrel + rod:.3f} m shorter than longest pin distance '
        f'{longest:.3f} m (rod pulls out of the barrel)')


def test_boom_cylinders_are_mirror_images(robot):
    left, *_ = pin_distances(robot, 'boom_cyl_left')
    right, *_ = pin_distances(robot, 'boom_cyl_right')
    np.testing.assert_allclose(left, right, atol=1e-9)
