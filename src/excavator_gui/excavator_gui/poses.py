"""
Preset poses and smooth motion between poses.

Angles are in radians and follow the URDF: positive raises the boom, arm and
bucket (an opened bucket tips its load out); negative swing turns right.
"""

import math

# Joint targets per preset. Joints a preset leaves out keep their position.
PRESET_POSES = {
    # Arm folded in, bucket curled and carried ~0.5 m above the ground in
    # front of the tracks.
    'Transport': {
        'swing_joint': 0.0,
        'boom_joint': 0.15,
        'arm_joint': -2.45,
        'bucket_joint': -1.65,
    },
    # Arm reaching out ~9 m, bucket opened, teeth just into the ground: the
    # start of a dig stroke.
    'Dig': {
        'swing_joint': 0.0,
        'boom_joint': 0.2,
        'arm_joint': -1.25,
        'bucket_joint': 0.85,
    },
    # Swung 90 degrees to the cab side, bucket ~3.8 m up (above a truck bed)
    # and tipped so the load falls out.
    'Dump': {
        'swing_joint': math.pi / 2,
        'boom_joint': 1.0,
        'arm_joint': -1.6,
        'bucket_joint': -0.2,
    },
}

# Speed for joints without a URDF velocity limit (rad/s or m/s).
DEFAULT_VELOCITY = 1.0
# Shortest move, so tiny moves are still visible as motion (s).
MIN_DURATION = 0.3
# Peak speed of the smoothstep profile relative to its average speed.
SMOOTHSTEP_PEAK = 1.5


def smoothstep(s):
    """Ease in and out: 0 -> 1 with zero speed at both ends."""
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


class Motion:
    """
    Move joints from start to target positions, all arriving together.

    The duration is chosen so that no joint exceeds its velocity limit at the
    peak of the smoothstep profile.
    """

    def __init__(self, start, target, velocities=None):
        velocities = velocities or {}
        self.start = dict(start)
        self.target = {name: value for name, value in target.items() if name in self.start}
        times = [
            abs(value - self.start[name]) / (velocities.get(name) or DEFAULT_VELOCITY)
            for name, value in self.target.items()
        ]
        self.duration = max([MIN_DURATION] + [SMOOTHSTEP_PEAK * t for t in times])

    def positions(self, elapsed):
        """Positions of the moving joints `elapsed` seconds after the start."""
        s = smoothstep(elapsed / self.duration)
        return {name: self.start[name] + (value - self.start[name]) * s
                for name, value in self.target.items()}

    def done(self, elapsed):
        return elapsed >= self.duration
