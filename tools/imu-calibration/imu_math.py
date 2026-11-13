"""Body IMU mount calibration: decode, foot-flat kinematics and the roll/pitch solve.

Conventions follow robotd 0.17.8 `duck-control/src/imu.rs`: quaternions are scalar-first
`[w, x, y, z]`; the mount M maps sensor vectors into the trunk frame; the trunk frame is
x forward, y left, z up, and upright gravity is `[0, 0, -1]`. A correction C is applied on the
left, M' = C ⊗ M, so robotd's gravity becomes C · g.
"""
from __future__ import annotations
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

# Wire order of robotd's JOINT_NAMES / JOINT_IDS.
JOINT_IDS = (20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14)
JOINT_NAMES = ('left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee', 'left_ankle',
               'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll', 'mouth',
               'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee', 'right_ankle')
IMU_ID, IMU_ADDRESS, IMU_LENGTH = 200, 124, 12
# robotd's DEFAULT_POSITION, the training HOME_FRAME.
HOME = (0.0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0.0, 0.0, 0.0,
        0.0, 0.0873, 0.4579, 0.0049, -0.4530)

# SflpDecoder::DEFAULT_MOUNT (zero3, +90° about Y) and BETA_MOUNT.
MOUNTS = {'zero3': (math.sqrt(.5), 0.0, math.sqrt(.5), 0.0), 'beta': (1.0, 0.0, 0.0, 0.0)}
GYRO_RAD_PER_LSB = math.pi / (180 * 16.4)
MJCF = Path(__file__).resolve().parents[2] / 'assets/models/robot_walk.xml'


def mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)


def conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def normalise(v):
    n = math.sqrt(sum(c*c for c in v))
    return tuple(c/n for c in v)


def rotate(q, v):
    return mul(mul(q, (0.0, *v)), conj(q))[1:]


def rotate_inverse(q, v):
    return rotate(conj(q), v)


def axis_angle(v):
    """Rotation vector (radians) → unit quaternion."""
    angle = math.sqrt(sum(c*c for c in v))
    if angle < 1e-15:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(angle/2)/angle
    return (math.cos(angle/2), v[0]*s, v[1]*s, v[2]*s)


def rotation_vector(q):
    q = q if q[0] >= 0 else tuple(-c for c in q)
    s = math.sqrt(q[1]**2 + q[2]**2 + q[3]**2)
    if s < 1e-15:
        return (0.0, 0.0, 0.0)
    angle = 2*math.atan2(s, q[0])
    return tuple(c/s*angle for c in q[1:])


def angle_between(a, b):
    a, b = normalise(a), normalise(b)
    cross = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    return math.atan2(math.sqrt(sum(c*c for c in cross)), sum(x*y for x, y in zip(a, b)))


def tilt_deg(g):
    """Forward and right-side-down tilt of the trunk that produces gravity g, degrees."""
    return (math.degrees(math.atan2(g[0], -g[2])), math.degrees(math.atan2(-g[1], -g[2])))


def half(bits):
    """IEEE 754 binary16 → float, as the LSM6DSV16X SFLP block ships its quaternion."""
    sign = -1.0 if bits & 0x8000 else 1.0
    exp, frac = (bits >> 10) & 0x1F, bits & 0x3FF
    if exp == 0:
        return sign*frac*2.0**-24
    if exp == 0x1F:
        return sign*math.inf if frac == 0 else math.nan
    return sign*(1 + frac/1024)*2.0**(exp - 15)


def decode_block(raw, mount):
    """The board's 12-byte block → (trunk gyro rad/s, trunk gravity), as robotd decodes it.

    Returns None for the gravity when SFLP has not written a quaternion yet.
    """
    raw = bytes(raw)
    if len(raw) != IMU_LENGTH:
        raise ValueError(f'IMU 数据块应为 {IMU_LENGTH} 字节，收到 {len(raw)}')
    counts = [int.from_bytes(raw[i:i+2], 'little', signed=True) for i in (0, 2, 4)]
    gyro = rotate(mount, [c*GYRO_RAD_PER_LSB for c in counts])
    packed = [int.from_bytes(raw[i:i+2], 'little') for i in (6, 8, 10)]
    if packed == [0, 0, 0]:
        return gyro, None
    x, y, z = (half(p) for p in packed)
    norm_sq = x*x + y*y + z*z
    if not all(math.isfinite(c) for c in (x, y, z)) or norm_sq > 1.02:
        return gyro, None
    q = mul((math.sqrt(max(0.0, 1 - norm_sq)), x, y, z), conj(mount))
    return gyro, normalise(rotate_inverse(normalise(q), (0.0, 0.0, -1.0)))


def counts_to_radians(position, trim):
    """robotd's read: 2π·count/4096 − π − trim."""
    return 2*math.pi*position/4096 - math.pi - trim*2*math.pi/4096


def read_zero_trim(path):
    """ZERO_TRIM_COUNTS from robotd's model.rs or a workbench *-trims.txt, in wire order."""
    text = Path(path).read_text(encoding='utf-8')
    match = re.search(r'ZERO_TRIM_COUNTS\s*:[^=]*=\s*\[(.*?)\];', text, re.S)
    if not match:
        raise ValueError(f'{path} 里没有 ZERO_TRIM_COUNTS 数组')
    body = re.sub(r'//[^\n]*', '', match.group(1))
    values = [int(v) for v in re.findall(r'-?\d+', body)]
    if len(values) != len(JOINT_IDS):
        raise ValueError(f'ZERO_TRIM_COUNTS 应有 {len(JOINT_IDS)} 项，读到 {len(values)}')
    return values


class Model:
    """Orientation-only FK over the MJCF tree, rooted at trunk_base (the IMU site's frame)."""

    def __init__(self, path=MJCF):
        root = ET.parse(path).getroot()
        self.trunk = root.find('.//body[@name="trunk_base"]')
        if self.trunk is None:
            raise ValueError('MJCF 缺少 trunk_base')
        imu = self.trunk.find('site[@name="imu"]')
        if imu is None or self._quat(imu) != (1.0, 0.0, 0.0, 0.0):
            raise ValueError('MJCF 的 imu 站点应与 trunk_base 同向')

    @staticmethod
    def _quat(element):
        # MJCF writes 0.707107 for √½; renormalise so the chain stays a rotation.
        q = normalise([float(v) for v in (element.get('quat') or '1 0 0 0').split()])
        return tuple(0.0 if c == 0 else c for c in q)

    def site(self, name, angles):
        """Orientation of `name` in the trunk frame; angles keyed by joint name."""
        def walk(body, q):
            for site in body.findall('site'):
                if site.get('name') == name:
                    return mul(q, self._quat(site))
            for child in body.findall('body'):
                qc = mul(q, self._quat(child))
                joint = child.find('joint')
                if joint is not None:
                    axis = [float(v) for v in (joint.get('axis') or '0 0 1').split()]
                    qc = mul(qc, axis_angle([c*angles.get(joint.get('name'), 0.0) for c in axis]))
                found = walk(child, qc)
                if found:
                    return found
            return None
        found = walk(self.trunk, (1.0, 0.0, 0.0, 0.0))
        if found is None:
            raise ValueError(f'MJCF 没有站点 {name}')
        return found

    def foot_gravity(self, angles):
        """Trunk-frame gravity if each sole lies flat on level ground: (left, right).

        The foot site's z is the sole normal. At HOME's ±5° hip roll the soles are therefore
        not flat with the trunk level: a robot standing stiff at HOME on a table measured its
        hip rolls pushed back to about 0°, which is where this model puts both soles flat.
        """
        return tuple(rotate(self.site(foot, angles), (0.0, 0.0, -1.0)) for foot in ('left_foot', 'right_foot'))


def expected_gravity(model, positions, trims):
    """Kinematic gravity from raw positions: mean of both feet, their disagreement (rad), and
    each foot's own (left, right)."""
    return expected_gravity_from_angles(model, [counts_to_radians(p, t) for p, t in zip(positions, trims)])


def expected_gravity_from_angles(model, radians):
    """As expected_gravity, from joint angles in wire order (robotd's `joints`)."""
    left, right = model.foot_gravity(dict(zip(JOINT_NAMES, radians)))
    return normalise([a + b for a, b in zip(left, right)]), angle_between(left, right), (left, right)


def solve_roll_pitch(pairs, iterations=20):
    """Rotation vector (x, y, 0) of C minimising Σ|C·measured − expected|².

    Yaw about the trunk's up is left at zero: with the trunk near upright it does not move
    gravity, and neither the policy nor fall detection can see it.
    """
    if not pairs:
        raise ValueError('没有样本')
    c = (1.0, 0.0, 0.0, 0.0)
    for _ in range(iterations):
        a = [[0.0, 0.0], [0.0, 0.0]]
        b = [0.0, 0.0]
        for measured, expected in pairs:
            g = rotate(c, measured)
            r = [e - x for e, x in zip(expected, g)]
            columns = ((0.0, -g[2], g[1]), (g[2], 0.0, -g[0]))
            for i in range(2):
                b[i] += sum(u*v for u, v in zip(columns[i], r))
                for j in range(2):
                    a[i][j] += sum(u*v for u, v in zip(columns[i], columns[j]))
        det = a[0][0]*a[1][1] - a[0][1]*a[1][0]
        if abs(det) < 1e-12:
            raise ValueError('样本无法确定俯仰和横滚')
        step = ((a[1][1]*b[0] - a[0][1]*b[1])/det, (a[0][0]*b[1] - a[1][0]*b[0])/det)
        c = normalise(mul(axis_angle((step[0], step[1], 0.0)), c))
        if math.hypot(*step) < 1e-12:
            break
    v = rotation_vector(c)
    return (v[0], v[1], 0.0)


def solve(captures, mount):
    """Correction from captures; averages headings 0 and 180 so a tilted table cancels.

    Each capture is a dict with `heading` (0 or 180), `measured` and `expected` gravity.
    """
    groups = {}
    for capture in captures:
        groups.setdefault(capture['heading'], []).append((capture['measured'], capture['expected']))
    unknown = set(groups) - {0, 180}
    if unknown:
        raise ValueError(f'朝向只能是 0 或 180，收到 {sorted(unknown)}')
    per_heading = {h: solve_roll_pitch(p) for h, p in sorted(groups.items())}
    if len(per_heading) == 2:
        v0, v180 = per_heading[0], per_heading[180]
        vector = tuple((a + b)/2 for a, b in zip(v0, v180))
        # Each heading's solve also undoes the table; half the difference is the table itself.
        table = tuple((b - a)/2 for a, b in zip(v0, v180))
    else:
        vector, table = next(iter(per_heading.values())), None
    correction = axis_angle(vector)
    residuals = [math.degrees(angle_between(rotate(correction, c['measured']), c['expected'])) for c in captures]
    return dict(
        correction=correction,
        correction_deg=[math.degrees(c) for c in vector[:2]],
        per_heading_deg={str(h): [math.degrees(c) for c in v[:2]] for h, v in per_heading.items()},
        table_tilt_deg=None if table is None else [math.degrees(c) for c in table[:2]],
        mount=normalise(mul(correction, mount)),
        residuals_deg=residuals)
