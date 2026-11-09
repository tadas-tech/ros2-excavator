"""
TF publisher for hydraulic cylinders.

Finds every cylinder in the URDF on /robot_description: a pair of links
<name>_barrel and <name>_rod attached by floating joints. The barrel's joint
origin is its mounting pin on the base link, the rod's joint origin is its
pin on the tip link. Whenever the base -> tip transform changes, both links
are re-published so the barrel's and the rod's +X axes point at each other's
pin, which makes the cylinder extend and retract with the joints.
"""

from dataclasses import dataclass
import xml.etree.ElementTree as ET

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

PUBLISH_RATE_HZ = 30.0


@dataclass
class Cylinder:
    name: str
    base: str            # link the barrel hangs from
    base_pin: np.ndarray  # barrel pin in the base frame
    tip: str             # link the rod hangs from
    tip_pin: np.ndarray   # rod pin in the tip frame
    last_stamp: tuple = None


def parse_cylinders(urdf_xml):
    floating = {}
    for joint in ET.fromstring(urdf_xml).iter('joint'):
        if joint.get('type') != 'floating':
            continue
        origin = joint.find('origin')
        xyz = origin.get('xyz', '0 0 0') if origin is not None else '0 0 0'
        floating[joint.find('child').get('link')] = (
            joint.find('parent').get('link'), np.array([float(v) for v in xyz.split()]))

    cylinders = []
    for child, (parent, pin) in floating.items():
        if not child.endswith('_barrel'):
            continue
        name = child[:-len('_barrel')]
        if name + '_rod' not in floating:
            continue
        tip, tip_pin = floating[name + '_rod']
        cylinders.append(Cylinder(name, parent, pin, tip, tip_pin))
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

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self._on_description, qos)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)

    def _on_description(self, msg):
        self.cylinders = parse_cylinders(msg.data)
        self.get_logger().info(
            'Cylinders: ' + (', '.join(c.name for c in self.cylinders) or 'none'))

    def _publish(self):
        transforms = []
        for cyl in self.cylinders:
            try:
                t = self.tf_buffer.lookup_transform(cyl.base, cyl.tip, Time())
            except TransformException:
                continue
            stamp = (t.header.stamp.sec, t.header.stamp.nanosec)
            if stamp == cyl.last_stamp:
                continue  # joints unchanged; re-sending would be repeated TF data
            cyl.last_stamp = stamp

            # base_T_tip
            r = t.transform.rotation
            R = quat_to_matrix(r.x, r.y, r.z, r.w)
            p = np.array([t.transform.translation.x, t.transform.translation.y,
                          t.transform.translation.z])
            tip_pin_in_base = R @ cyl.tip_pin + p
            base_pin_in_tip = R.T @ (cyl.base_pin - p)

            transforms.append(self._make_tf(
                t.header.stamp, cyl.base, cyl.name + '_barrel',
                cyl.base_pin, tip_pin_in_base - cyl.base_pin))
            transforms.append(self._make_tf(
                t.header.stamp, cyl.tip, cyl.name + '_rod',
                cyl.tip_pin, base_pin_in_tip - cyl.tip_pin))
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)

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
