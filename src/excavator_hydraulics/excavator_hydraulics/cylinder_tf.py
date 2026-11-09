"""
TF and state publisher for hydraulic cylinders.

Finds every cylinder in the URDF on /robot_description: a pair of links
<name>_barrel and <name>_rod attached by floating joints. The barrel's joint
origin is its mounting pin on the base link, the rod's joint origin is its
pin on the tip link. Whenever the base -> tip transform changes, both links
are re-published so the barrel's and the rod's +X axes point at each other's
pin, which makes the cylinder extend and retract with the joints.

It also publishes each cylinder's pin-to-pin length, stroke and speed on
/cylinder_states (excavator_msgs/CylinderStates).
"""

from dataclasses import dataclass
import math
import time
import xml.etree.ElementTree as ET

from excavator_msgs.msg import CylinderState, CylinderStates
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

PUBLISH_RATE_HZ = 30.0
RANGE_SAMPLES = 2001  # joint positions checked when finding a cylinder's length range
VELOCITY_TIMEOUT = 0.2  # s without a new transform before velocity reads 0


@dataclass
class Cylinder:
    name: str
    base: str             # link the barrel hangs from
    base_pin: np.ndarray  # barrel pin in the base frame
    tip: str              # link the rod hangs from
    tip_pin: np.ndarray   # rod pin in the tip frame
    min_length: float = math.nan  # pin-to-pin length range over the joint's limits
    max_length: float = math.nan
    last_stamp: tuple = None      # stamp of the last base -> tip transform used
    length: float = math.nan
    velocity: float = 0.0
    last_change: float = 0.0      # monotonic time the transform last changed


def _vec(element, attr, default='0 0 0'):
    value = element.get(attr, default) if element is not None else default
    return np.array([float(v) for v in value.split()])


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def length_range(joint, base_pin, tip_pin):
    """
    Shortest and longest pin-to-pin length as `joint` moves over its range.

    `joint` is the URDF <joint> element joining the cylinder's base link
    (parent) to its tip link (child). Returns (nan, nan) for other joint types.
    """
    jtype = joint.get('type')
    if jtype == 'continuous':
        lower, upper = -math.pi, math.pi
    elif jtype == 'revolute':
        limit = joint.find('limit')
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
    else:
        return math.nan, math.nan

    origin = joint.find('origin')
    offset = _vec(origin, 'xyz')
    fixed = rpy_to_matrix(*_vec(origin, 'rpy'))
    axis = _vec(joint.find('axis'), 'xyz', '1 0 0')
    k = axis / np.linalg.norm(axis)

    # Rodrigues rotation of tip_pin about k, for every sampled angle at once.
    angles = np.linspace(lower, upper, RANGE_SAMPLES)[:, None]
    rotated = (tip_pin * np.cos(angles) + np.cross(k, tip_pin) * np.sin(angles)
               + np.outer(1 - np.cos(angles), k * (k @ tip_pin)))
    lengths = np.linalg.norm(offset + rotated @ fixed.T - base_pin, axis=1)
    return float(lengths.min()), float(lengths.max())


def parse_cylinders(urdf_xml):
    joints = list(ET.fromstring(urdf_xml).iter('joint'))
    by_child = {j.find('child').get('link'): j for j in joints}

    floating = {}
    for joint in joints:
        if joint.get('type') != 'floating':
            continue
        floating[joint.find('child').get('link')] = (
            joint.find('parent').get('link'), _vec(joint.find('origin'), 'xyz'))

    cylinders = []
    for child, (parent, pin) in floating.items():
        if not child.endswith('_barrel'):
            continue
        name = child[:-len('_barrel')]
        if name + '_rod' not in floating:
            continue
        tip, tip_pin = floating[name + '_rod']
        cyl = Cylinder(name, parent, pin, tip, tip_pin)
        joint = by_child.get(tip)
        if joint is not None and joint.find('parent').get('link') == parent:
            cyl.min_length, cyl.max_length = length_range(joint, pin, tip_pin)
        cylinders.append(cyl)
    return cylinders


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def quat_x_to(v):
    """Quaternion (x, y, z, w) rotating the +X axis onto direction v."""
    v = v / np.linalg.norm(v)
    axis = np.array([0.0, -v[2], v[1]])  # (1, 0, 0) x v
    s = np.linalg.norm(axis)
    if s < 1e-9:
        return (0.0, 0.0, 0.0, 1.0) if v[0] > 0 else (0.0, 0.0, 1.0, 0.0)
    angle = np.arctan2(s, v[0])
    axis = axis / s * np.sin(angle / 2)
    return (axis[0], axis[1], axis[2], np.cos(angle / 2))


class CylinderTf(Node):

    def __init__(self):
        super().__init__('excavator_cylinder_tf')
        self.cylinders = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self._on_description, latched)
        self.state_pub = self.create_publisher(CylinderStates, 'cylinder_states', latched)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)

    def _on_description(self, msg):
        self.cylinders = parse_cylinders(msg.data)
        for c in self.cylinders:
            self.get_logger().info(
                f'Cylinder {c.name}: {c.base} -> {c.tip}, '
                f'length {c.min_length:.3f} .. {c.max_length:.3f} m')
        if not self.cylinders:
            self.get_logger().info('Cylinders: none')

    def _publish(self, now=None):
        now = time.monotonic() if now is None else now
        transforms = []
        for cyl in self.cylinders:
            try:
                t = self.tf_buffer.lookup_transform(cyl.base, cyl.tip, Time())
            except TransformException:
                continue
            stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
            if stamp == cyl.last_stamp:
                # No new joint state. Keep the speed through short gaps between
                # updates, but report 0 once the joints have stopped publishing.
                if now - cyl.last_change > VELOCITY_TIMEOUT:
                    cyl.velocity = 0.0
                continue  # re-sending would be repeated TF data

            # base_T_tip
            r = t.transform.rotation
            R = quat_to_matrix(r.x, r.y, r.z, r.w)
            p = np.array([t.transform.translation.x, t.transform.translation.y,
                          t.transform.translation.z])
            tip_pin_in_base = R @ cyl.tip_pin + p
            base_pin_in_tip = R.T @ (cyl.base_pin - p)

            length = float(np.linalg.norm(tip_pin_in_base - cyl.base_pin))
            if cyl.last_stamp is not None:
                dt = (stamp[0] - cyl.last_stamp[0]) + (stamp[1] - cyl.last_stamp[1]) * 1e-9
                cyl.velocity = (length - cyl.length) / dt if dt > 0 else 0.0
            cyl.length = length
            cyl.last_stamp = stamp
            cyl.last_change = now

            transforms.append(self._make_tf(
                t.header.stamp, cyl.base, cyl.name + '_barrel',
                cyl.base_pin, tip_pin_in_base - cyl.base_pin))
            transforms.append(self._make_tf(
                t.header.stamp, cyl.tip, cyl.name + '_rod',
                cyl.tip_pin, base_pin_in_tip - cyl.tip_pin))
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
        self._publish_states()

    def _publish_states(self):
        known = [c for c in self.cylinders if c.last_stamp is not None]
        if not known:
            return
        msg = CylinderStates()
        msg.header.stamp = self.get_clock().now().to_msg()
        for c in known:
            msg.cylinders.append(CylinderState(
                name=c.name, length=c.length, min_length=c.min_length,
                max_length=c.max_length, stroke=c.length - c.min_length,
                velocity=c.velocity))
        self.state_pub.publish(msg)

    @staticmethod
    def _make_tf(stamp, parent, child, origin, direction):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x, msg.transform.translation.y, \
            msg.transform.translation.z = (float(v) for v in origin)
        q = quat_x_to(direction)
        msg.transform.rotation.x, msg.transform.rotation.y, \
            msg.transform.rotation.z, msg.transform.rotation.w = (float(v) for v in q)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CylinderTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
