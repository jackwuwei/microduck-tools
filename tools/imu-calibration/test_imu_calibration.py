"""Offline tests: decode, kinematics and solve against synthetic poses; SSH worker end to end."""
import inspect
import math
from pathlib import Path
import random
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import imu_math
from imu_math import MOUNTS, Model, axis_angle, conj, mul, rotate, rotate_inverse
import imu_calibration
from imu_bus import ImuSshBus, imu_remote_worker, snapshot, worker_config
from ssh_bus import SshBus

from imu_math import HOME
# HOME as it stands on a table: the hip rolls pushed back to 0° by the floor.
STANDING = tuple(0.0 if i in (1, 11) else a for i, a in enumerate(HOME))
MODEL = Model()


def deg(*v):
    return [math.radians(c) for c in v]


def counts(angles, trims):
    return [round((a + math.pi)*4096/(2*math.pi) + t) for a, t in zip(angles, trims)]


def block(gyro_counts, trunk_to_world, mount):
    """Encode what the board would send for a trunk attitude under `mount`."""
    sensor = mul(trunk_to_world, mount)
    if sensor[0] < 0:
        sensor = tuple(-c for c in sensor)
    raw = struct.pack('<3h', *gyro_counts) + struct.pack('<3e', *sensor[1:])
    return list(raw)


def flat_pose(rng):
    """A random limp pose with both soles in one plane, and the trunk attitude it implies."""
    while True:
        angles = list(HOME)
        for i in (2, 3, 4, 12, 13, 14):
            angles[i] += rng.uniform(-.25, .25)
        angles[1] = angles[11] = 0.0
        angles[0] = angles[10] = 0.0
        # Mirror the right leg so both soles share one plane.
        for left, right in ((2, 12), (3, 13), (4, 14)):
            angles[right] = -angles[left]
        left, right = MODEL.foot_gravity(dict(zip(imu_math.JOINT_NAMES, angles)))
        if imu_math.angle_between(left, right) < 1e-6:
            return angles, imu_math.normalise(left)


def attitude_for(gravity):
    """A trunk→world quaternion whose trunk-frame gravity is `gravity` (yaw arbitrary)."""
    down = (0.0, 0.0, -1.0)
    axis = [down[1]*gravity[2]-down[2]*gravity[1], down[2]*gravity[0]-down[0]*gravity[2],
            down[0]*gravity[1]-down[1]*gravity[0]]
    angle = imu_math.angle_between(down, gravity)
    n = math.sqrt(sum(c*c for c in axis))
    q = axis_angle([c/n*angle for c in axis]) if n > 1e-12 else (1.0, 0.0, 0.0, 0.0)
    return conj(q)  # rotate_inverse(q, down) == gravity


class MathTests(unittest.TestCase):
    def test_half_precision(self):
        self.assertEqual(imu_math.half(0x3C00), 1.0)
        self.assertEqual(imu_math.half(0xC000), -2.0)
        self.assertEqual(imu_math.half(0x0001), 2.0**-24)
        self.assertTrue(math.isnan(imu_math.half(0x7E00)))

    def test_decode_matches_robotd_convention(self):
        # Upright robot on the zero3 mount: trunk gravity is straight down.
        gyro, gravity = imu_math.decode_block(block((164, 0, 0), (1, 0, 0, 0), MOUNTS['zero3']), MOUNTS['zero3'])
        for got, want in zip(gravity, (0, 0, -1)):
            self.assertAlmostEqual(got, want, places=3)
        # 164 counts = 10 °/s about sensor x, which is trunk −z on zero3.
        self.assertAlmostEqual(math.degrees(gyro[2]), -10.0, places=6)
        tilted = axis_angle(deg(0, 10, 0))  # nose down 10°
        _, gravity = imu_math.decode_block(block((0, 0, 0), tilted, MOUNTS['zero3']), MOUNTS['zero3'])
        self.assertAlmostEqual(imu_math.tilt_deg(gravity)[0], 10.0, places=1)
        self.assertIsNone(imu_math.decode_block([0]*12, MOUNTS['zero3'])[1])

    def test_home_hip_roll_rolls_the_soles_and_cancels_on_average(self):
        left, right = MODEL.foot_gravity(dict(zip(imu_math.JOINT_NAMES, HOME)))
        self.assertAlmostEqual(abs(imu_math.tilt_deg(left)[1]), 5.0, places=1)
        self.assertAlmostEqual(imu_math.tilt_deg(left)[1], -imu_math.tilt_deg(right)[1], places=6)
        self.assertLess(abs(imu_math.tilt_deg(left)[0]), .5)
        mean = imu_math.expected_gravity_from_angles(MODEL, HOME)[0]
        for got, want in zip(mean, (0, 0, -1)):
            self.assertAlmostEqual(got, want, places=6)

    def test_zero_trim_from_model_rs_and_workbench_file(self):
        text = '''pub const ZERO_TRIM_COUNTS: [i32; NUM_JOINTS] = [
    -7, -4, 3, 14, 9, // left leg: 20 21 22 23 24
    -24, -24, -15, -59, 0, // neck, head, mouth: 30 31 32 33 34
    5, -22, -40, -13, -4, // right leg: 10 11 12 13 14
];'''
        with tempfile.NamedTemporaryFile('w', suffix='.rs') as file:
            file.write('ID 20 2041 -25\n' + text)
            file.flush()
            self.assertEqual(imu_math.read_zero_trim(file.name),
                             [-7, -4, 3, 14, 9, -24, -24, -15, -59, 0, 5, -22, -40, -13, -4])
        self.assertAlmostEqual(imu_math.counts_to_radians(2048 + 11, 11), 0.0)

    def test_solve_recovers_mount_error_and_cancels_table_tilt(self):
        rng = random.Random(7)
        error = axis_angle(deg(1.8, -8.5, 0))      # what C must undo
        table = deg(.6, -.9, 0)                     # world tilt, in the heading-0 trunk frame
        captures = []
        for heading in (0, 180, 0, 180, 0, 180, 0, 180):
            _, expected = flat_pose(rng)
            sign = 1 if heading == 0 else -1
            truth = rotate(axis_angle([sign*c for c in table]), expected)
            measured = rotate_inverse(error, truth)
            captures.append(dict(heading=heading, measured=measured, expected=expected))
        result = imu_math.solve(captures, MOUNTS['zero3'])
        self.assertAlmostEqual(result['correction_deg'][0], 1.8, places=1)
        self.assertAlmostEqual(result['correction_deg'][1], -8.5, places=1)
        self.assertAlmostEqual(result['table_tilt_deg'][0], .6, places=1)
        self.assertAlmostEqual(result['table_tilt_deg'][1], -.9, places=1)
        # A board read through the new mount reports the true gravity.
        _, expected = flat_pose(rng)
        raw = block((0, 0, 0), attitude_for(rotate_inverse(error, expected)), MOUNTS['zero3'])
        # The board's sensor attitude is fixed; robotd re-reads it through the corrected mount.
        _, gravity = imu_math.decode_block(raw, MOUNTS['zero3'])
        corrected = rotate(result['correction'], gravity)
        self.assertLess(math.degrees(imu_math.angle_between(corrected, expected)), .1)
        self.assertLess(math.degrees(imu_math.angle_between(
            imu_math.decode_block(raw, result['mount'])[1], expected)), .1)


class FakeBus:
    def __init__(self, positions, raw, torque=0):
        self.positions, self.raw, self.torque = positions, raw, torque
    def read(self, motor, name):
        index = imu_math.JOINT_IDS.index(motor)
        return self.torque if name == 'torque' else self.positions[index]
    def read_block(self, motor, address, length):
        assert (motor, address, length) == (200, 124, 12)
        return list(self.raw)
    def snapshot(self, torque=False):
        return snapshot(self, imu_math.JOINT_IDS, (200, 124, 12), torque)


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.trims = [-7, -4, 3, 14, 9, -24, -24, -15, -59, 0, 5, -22, -40, -13, -4]
        angles, self.expected = flat_pose(random.Random(3))
        self.positions = counts(angles, self.trims)
        self.raw = block((0, 0, 0), attitude_for(self.expected), MOUNTS['zero3'])
        patcher = patch('imu_calibration.time.sleep')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_flat_pose_reads_its_own_attitude(self):
        capture = imu_calibration.take(FakeBus(self.positions, self.raw), MODEL, self.trims, MOUNTS['zero3'], 0)
        self.assertLess(capture['feet_disagreement_deg'], .2)
        self.assertLess(math.degrees(imu_math.angle_between(capture['measured'], capture['expected'])), .2)

    def test_rejects_torque_and_a_lifted_foot(self):
        with self.assertRaisesRegex(imu_calibration.Rejected, '扭矩'):
            imu_calibration.take(FakeBus(self.positions, self.raw, torque=1), MODEL, self.trims, MOUNTS['zero3'], 0)
        lifted = list(self.positions)
        lifted[4] += 80  # left ankle 7° off: the left sole is no longer flat
        with self.assertRaisesRegex(imu_calibration.Rejected, '平贴'):
            imu_calibration.take(FakeBus(lifted, self.raw), MODEL, self.trims, MOUNTS['zero3'], 0)

    def test_rejects_empty_quaternion(self):
        with self.assertRaisesRegex(imu_calibration.Rejected, 'SFLP'):
            imu_calibration.take(FakeBus(self.positions, [0]*12), MODEL, self.trims, MOUNTS['zero3'], 0)


class StandTests(unittest.TestCase):
    def frames(self, gravity, joints, limp=False, n=20):
        return [dict(joints=list(joints), imu=dict(gyro=[0.0, 0.0, 0.0]),
                     safety=dict(gravity=list(gravity), limp=limp, fallen=False)) for _ in range(n)]

    def test_robotd_frames_give_the_mount_error(self):
        error = axis_angle(deg(.5, -3.0, 0))
        expected = imu_math.expected_gravity_from_angles(MODEL, STANDING)[0]
        capture = imu_calibration.take_stand(
            self.frames(rotate_inverse(error, expected), STANDING), MODEL, 0)
        self.assertAlmostEqual(capture['feet_disagreement_deg'], 0.0, places=6)
        result = imu_math.solve([capture], MOUNTS['zero3'])
        self.assertAlmostEqual(result['correction_deg'][0], .5, places=2)
        self.assertAlmostEqual(result['correction_deg'][1], -3.0, places=2)

    def test_rejects_limp_and_moving(self):
        with self.assertRaisesRegex(imu_calibration.Rejected, '卸力'):
            imu_calibration.take_stand(self.frames((0, 0, -1), STANDING, limp=True), MODEL, 0)
        frames = self.frames((0, 0, -1), STANDING)
        frames[0]['joints'][3] += .05
        with self.assertRaisesRegex(imu_calibration.Rejected, '关节在动'):
            imu_calibration.take_stand(frames, MODEL, 0)


class FakeImuSshBus(ImuSshBus):
    def _program(self):
        source = (Path(__file__).resolve().parents[1] / 'xl330-calibration/calibration.py').read_text()
        source += '''
class ImuSerialBus(DemoBus):
    def __init__(self, *args):
        super().__init__()
    def read_block(self, motor, address, length):
        return list(range(length))
'''
        config = worker_config('fake', 1_000_000, True, [])
        return '\n'.join([source, inspect.getsource(snapshot), inspect.getsource(imu_remote_worker),
                          f'imu_remote_worker({config!r})']) + '\n'


class SshTests(unittest.TestCase):
    def setUp(self):
        real_popen = subprocess.Popen
        def local_worker(command, **kwargs):
            return real_popen([sys.executable, '-u', '-c', SshBus.BOOTSTRAP], **kwargs)
        self.patch = patch('ssh_bus.subprocess.Popen', side_effect=local_worker)
        self.patch.start()
        self.bus = FakeImuSshBus(timeout=2)
    def tearDown(self):
        self.bus.close()
        self.patch.stop()

    def test_snapshot_over_stdio(self):
        self.bus.open()
        value = self.bus.snapshot(torque=True)
        self.assertEqual(value['positions'], [2048]*15)
        self.assertEqual(value['imu'], list(range(12)))
        self.assertEqual(value['torque'], [0]*15)
        self.assertNotIn('torque', self.bus.snapshot())

    def test_worker_refuses_anything_but_snapshot(self):
        self.bus.open()
        with self.assertRaisesRegex(ValueError, '未知'):
            self.bus._request('write', motor=20, name='offset', value=0)
        self.assertIsNone(self.bus.process)

    def test_real_program_ships_the_serial_bus(self):
        program = ImuSshBus()._program()
        self.assertIn('class ImuSerialBus(SerialBus)', program)
        self.assertIn("'imu': [200, 124, 12]", program)
        compile(program, 'remote', 'exec')


if __name__ == '__main__':
    unittest.main()
