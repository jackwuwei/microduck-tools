"""Read-only bus access for the IMU calibration: servo positions and the IMU board's block.

Reuses the XL330 workbench's SerialBus (exclusive open, silent-bus check, 2S alert handling)
and its SSH stdio transport. Nothing here writes a register.
"""
import inspect
from pathlib import Path
import sys

WORKBENCH = Path(__file__).resolve().parents[1] / 'xl330-calibration'
sys.path.insert(0, str(WORKBENCH))

from calibration import SerialBus  # noqa: E402
from ssh_bus import SshBus  # noqa: E402
import imu_math  # noqa: E402


class ImuSerialBus(SerialBus):
    def read_block(self, motor, address, length):
        if self.port.ser.in_waiting:
            raise ValueError('发现其他总线流量，停止会话')
        data, result, error = self.packet.readTxRx(self.port, motor, address, length)
        self.check(result, error)
        if len(data) != length:
            raise ValueError(f'ID {motor} 返回 {len(data)} 字节，应为 {length}')
        return list(data)


def snapshot(bus, ids, imu, torque=False):
    """One reading: positions in wire order, the IMU block, and optionally torque flags."""
    out = dict(positions=[bus.read(motor, 'position') for motor in ids],
               imu=bus.read_block(*imu))
    if torque:
        out['torque'] = [bus.read(motor, 'torque') for motor in ids]
    return out


def imu_remote_worker(config):
    """Runs in memory on the controller; answers read-only snapshot requests."""
    import json
    import os
    import sys
    for path in reversed(config['pythonpath']):
        sys.path.insert(0, os.path.expanduser(path))
    bus = None
    try:
        bus = ImuSerialBus(config['serial'], config['baud'], config['microduck_2s'])
        bus.open()
        print(json.dumps({'ok': True, 'value': None}), flush=True)
        for line in sys.stdin.buffer:
            try:
                request = json.loads(line)
                if request.get('operation') != 'snapshot':
                    raise ValueError('未知串口操作')
                value = snapshot(bus, config['ids'], config['imu'], request.get('torque') is True)
                print(json.dumps({'ok': True, 'value': value}), flush=True)
            except Exception as error:
                print(json.dumps({'ok': False, 'error': str(error)}), flush=True)
                break
    except Exception as error:
        print(json.dumps({'ok': False, 'error': str(error)}), flush=True)
    finally:
        if bus is not None:
            bus.close()


def worker_config(port, baud, microduck_2s, pythonpath):
    return dict(serial=port, baud=baud, microduck_2s=microduck_2s, pythonpath=pythonpath,
                ids=list(imu_math.JOINT_IDS),
                imu=[imu_math.IMU_ID, imu_math.IMU_ADDRESS, imu_math.IMU_LENGTH])


class ImuSshBus(SshBus):
    def _program(self):
        source = (WORKBENCH / 'calibration.py').read_text()
        config = worker_config(self.port, self.baud, self.microduck_2s, self.pythonpath)
        parts = [source, inspect.getsource(ImuSerialBus), inspect.getsource(snapshot),
                 inspect.getsource(imu_remote_worker), f'imu_remote_worker({config!r})']
        return '\n'.join(parts) + '\n'

    def snapshot(self, torque=False):
        return self._request('snapshot', torque=torque)


class ImuLocalBus(ImuSerialBus):
    def snapshot(self, torque=False):
        return snapshot(self, imu_math.JOINT_IDS,
                        (imu_math.IMU_ID, imu_math.IMU_ADDRESS, imu_math.IMU_LENGTH), torque)
