"""
Bucket-tip kinematics for the excavator's digging linkage.

Boom, arm and bucket swing in one vertical plane (the swing plane), so the
tip of the bucket is set by three numbers:

  reach   distance from the swing axis, along the swing direction (m)
  height  height above the ground (m)
  angle   pitch of the bucket: the sum of the boom, arm and bucket angles
          (rad). 0 is the bucket attitude of the all-zero pose; positive
          tips the bucket open.

Forward kinematics turns joint angles into (reach, height, angle); inverse
kinematics solves the joint angles for a wanted (reach, height, angle) in
closed form: the bucket pivot follows from the tip and the angle, then boom
and arm angles from the triangle they form.
"""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

LINKAGE_JOINTS = ('boom_joint', 'arm_joint', 'bucket_joint')
TIP_LINK = 'bucket_tip'


def _vec(element, attr, default='0 0 0'):
    value = element.get(attr, default) if element is not None else default
    return [float(v) for v in value.split()]


def _link(joint, tag):
    """Link name of a joint's <parent> or <child>, or None if missing."""
    element = joint.find(tag)
    return element.get('link') if element is not None else None


def _rot(angle, x, z):
    """Rotate (x, z) by `angle` in the swing plane (positive raises +X towards +Z)."""
    c, s = math.cos(angle), math.sin(angle)
    return x * c - z * s, x * s + z * c


def _wrap(angle):
    """Angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class PlanarJoint:
    name: str
    offset: tuple   # (x, z) of this joint in the previous link's frame
    sign: float     # +1 for axis (0 -1 0), -1 for axis (0 1 0)
    lower: float
    upper: float


class PlanarArm:
    """Boom, arm and bucket as a planar chain, with the tip point on the bucket."""

    def __init__(self, joints, tip):
        self.joints = joints  # boom, arm, bucket
        self.tip = tip        # (x, z) of the tip in the bucket frame

    @classmethod
    def from_urdf(cls, urdf_xml, joint_names=LINKAGE_JOINTS, tip_link=TIP_LINK):
        """
        Build from a URDF string.

        Raises ValueError if the joints don't form a chain of pitch joints
        ending in `tip_link`.
        """
        by_name, by_child = {}, {}
        for joint in ET.fromstring(urdf_xml).iter('joint'):
            by_name[joint.get('name')] = joint
            if _link(joint, 'child'):
                by_child[_link(joint, 'child')] = joint

        missing = [n for n in joint_names if n not in by_name]
        if missing or tip_link not in by_child:
            raise ValueError(f'URDF has no {", ".join(missing) or tip_link}')

        joints = []
        for i, name in enumerate(joint_names):
            joint = by_name[name]
            if not (_link(joint, 'parent') and _link(joint, 'child')):
                raise ValueError(f'{name} has no parent or child link')
            if i > 0:
                prev_child = _link(by_name[joint_names[i - 1]], 'child')
                if _link(joint, 'parent') != prev_child:
                    raise ValueError(f'{name} does not follow {joint_names[i - 1]}')
            axis = _vec(joint.find('axis'), 'xyz', '1 0 0')
            if abs(axis[0]) > 1e-9 or abs(axis[2]) > 1e-9 or abs(axis[1]) < 1e-9:
                raise ValueError(f'{name} does not pitch about Y')
            if any(abs(v) > 1e-9 for v in _vec(joint.find('origin'), 'rpy')):
                raise ValueError(f'{name} has a rotated origin')
            limit = joint.find('limit')
            if limit is None or limit.get('lower') is None or limit.get('upper') is None:
                raise ValueError(f'{name} has no position limits')
            x, _, z = _vec(joint.find('origin'), 'xyz')
            joints.append(PlanarJoint(
                name, (x, z), -1.0 if axis[1] > 0 else 1.0,
                float(limit.get('lower')), float(limit.get('upper'))))

        # Tip: fixed joints from the bucket link down to tip_link.
        tip_x, tip_z = 0.0, 0.0
        link = tip_link
        bucket = _link(by_name[joint_names[-1]], 'child')
        while link != bucket:
            joint = by_child.get(link)
            if joint is None or joint.get('type') != 'fixed':
                raise ValueError(f'{tip_link} is not fixed to {bucket}')
            x, _, z = _vec(joint.find('origin'), 'xyz')
            tip_x, tip_z = tip_x + x, tip_z + z
            link = _link(joint, 'parent')

        # Boom pivot in the swing frame: fixed offsets from the root down to the
        # boom joint (the swing joint turns about the vertical through x = y = 0).
        base_x, base_z = joints[0].offset
        link = _link(by_name[joint_names[0]], 'parent')
        while link in by_child:
            joint = by_child[link]
            x, _, z = _vec(joint.find('origin'), 'xyz')
            base_x, base_z = base_x + x, base_z + z
            link = _link(joint, 'parent')
        joints[0] = PlanarJoint(joints[0].name, (base_x, base_z), joints[0].sign,
                                joints[0].lower, joints[0].upper)
        return cls(joints, (tip_x, tip_z))

    # ------------------------------------------------------------ forward

    def forward(self, positions):
        """(reach, height, angle) of the tip for {joint name: position}."""
        x, z = self.joints[0].offset
        angle = 0.0
        for i, joint in enumerate(self.joints):
            if i > 0:
                dx, dz = _rot(angle, *joint.offset)
                x, z = x + dx, z + dz
            angle += joint.sign * positions.get(joint.name, 0.0)
        dx, dz = _rot(angle, *self.tip)
        return x + dx, z + dz, angle

    # ------------------------------------------------------------ inverse

    def inverse(self, reach, height, angle, current=None):
        """
        Joint positions that put the tip at (reach, height, angle).

        Of the (up to two) solutions within the joint limits, returns the one
        closest to `current` ({joint name: position}). None if out of reach.
        """
        boom, arm, bucket = self.joints
        # Bucket pivot from the tip and the bucket angle.
        tx, tz = _rot(angle, *self.tip)
        px, pz = reach - tx, height - tz
        # Triangle boom pivot -> arm pivot -> bucket pivot.
        dx, dz = px - boom.offset[0], pz - boom.offset[1]
        d = math.hypot(dx, dz)
        l1, b1 = math.hypot(*arm.offset), math.atan2(arm.offset[1], arm.offset[0])
        l2, b2 = math.hypot(*bucket.offset), math.atan2(bucket.offset[1], bucket.offset[0])
        cos_bend = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        if abs(cos_bend) > 1.0:
            return None

        solutions = []
        for bend in {math.acos(cos_bend), -math.acos(cos_bend)}:
            a1 = math.atan2(dz, dx) - math.atan2(l2 * math.sin(bend), l1 + l2 * math.cos(bend))
            a2 = a1 + bend
            t_boom = a1 - b1
            t_arm = a2 - b2 - t_boom
            t_bucket = angle - t_boom - t_arm
            q = {}
            for joint, theta in zip(self.joints, (t_boom, t_arm, t_bucket)):
                value = self._into_limits(joint, joint.sign * theta)
                if value is None:
                    break
                q[joint.name] = value
            else:
                solutions.append(q)
        if not solutions:
            return None
        current = current or {}
        return min(solutions, key=lambda q: sum(
            (q[n] - current.get(n, 0.0)) ** 2 for n in q))

    @staticmethod
    def _into_limits(joint, value):
        """`value` shifted by a whole turn into the joint's limits, or None."""
        value = _wrap(value)
        for candidate in (value, value - 2 * math.pi, value + 2 * math.pi):
            if joint.lower - 1e-9 <= candidate <= joint.upper + 1e-9:
                return min(max(candidate, joint.lower), joint.upper)
        return None
