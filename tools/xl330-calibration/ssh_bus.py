"""Local calibration bus using a temporary serial worker over SSH stdio."""
import inspect
import json
import os
from pathlib import Path
import re
import selectors
import shlex
import subprocess
import tempfile
import time


def remote_worker(config):
    """Runs in memory on the controller; no HTTP listener or deployed files."""
    import json
    import os
    import sys
    for path in reversed(config['pythonpath']):
        sys.path.insert(0, os.path.expanduser(path))
    bus = None
    try:
        bus = SerialBus(config['serial'], config['baud'], config['microduck_2s'])
        bus.open()
        print(json.dumps({'ok': True, 'value': None}), flush=True)
        for line in sys.stdin.buffer:
            try:
                request = json.loads(line)
                operation = request['operation']
                if operation == 'read_all':
                    value = {motor: {name: bus.read(motor, name) for name in REGISTERS} for motor in IDS}
                elif operation in ('read', 'write'):
                    motor, name = request['motor'], request['name']
                    if type(motor) is not int or motor not in IDS or name not in REGISTERS:
                        raise ValueError('无效关节或寄存器')
                    if operation == 'read':
                        value = bus.read(motor, name)
                    else:
                        if name not in WRITABLE or type(request['value']) is not int:
                            raise ValueError('该寄存器禁止写入或数值无效')
                        bus.write(motor, name, request['value'])
                        value = None
                else:
                    raise ValueError('未知串口操作')
                print(json.dumps({'ok': True, 'value': value}), flush=True)
            except Exception as error:
                print(json.dumps({'ok': False, 'error': str(error)}), flush=True)
                break
    except Exception as error:
        print(json.dumps({'ok': False, 'error': str(error)}), flush=True)
    finally:
        if bus is not None:
            bus.close()


class SshBus:
    # Bootstrap consumes one hex-encoded source line; remaining stdin is RPC.
    BOOTSTRAP = 'import sys; exec(bytes.fromhex(sys.stdin.buffer.readline().strip().decode()))'

    def __init__(self, host='microduck', port='/dev/ttyS2', baud=1_000_000,
                 microduck_2s=True, timeout=15, remote_python='python3', pythonpath=None):
        if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@:\-]*', host):
            raise ValueError('SSH 目标必须是主机名、SSH 别名或 user@host')
        self.host, self.port, self.baud = host, port, baud
        self.microduck_2s, self.timeout = microduck_2s, timeout
        self.remote_python = remote_python
        self.pythonpath = list(pythonpath) if pythonpath is not None else [
            '~/.local/share/microduck-serial/deps', '~/diagnostics/readlibs']
        self.process = None
        self.stderr = None
        self.buffer = b''

    def _program(self):
        source = Path(__file__).with_name('calibration.py').read_text()
        config = dict(serial=self.port, baud=self.baud, microduck_2s=self.microduck_2s,
                      pythonpath=self.pythonpath)
        return source + '\n' + inspect.getsource(remote_worker) + '\nremote_worker(' + repr(config) + ')\n'

    def open(self):
        remote = shlex.join([self.remote_python, '-u', '-c', self.BOOTSTRAP])
        self.stderr = tempfile.TemporaryFile()
        try:
            self.process = subprocess.Popen(
                ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
                 '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2', self.host, remote],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr)
            self._send_raw(self._program().encode().hex().encode() + b'\n')
            self._receive()
        except BaseException:
            self.close()
            raise

    def _send_raw(self, data):
        try:
            self.process.stdin.write(data)
            self.process.stdin.flush()
        except (BrokenPipeError, OSError) as error:
            raise ValueError('SSH 连接中断；请重新连接并核对备份') from error

    def _receive(self):
        deadline = time.monotonic() + self.timeout
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ)
            while b'\n' not in self.buffer:
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not selector.select(remaining):
                    raise ValueError('SSH 串口响应超时；已停止会话，请检查网络和主控')
                chunk = os.read(self.process.stdout.fileno(), 65536)
                if not chunk:
                    self.stderr.seek(0)
                    detail = self.stderr.read(2048).decode(errors='replace').strip()
                    raise ValueError('SSH 串口连接结束；请先在终端确认 ssh microduck 可登录。' + detail)
                self.buffer += chunk
                if len(self.buffer) > 1_000_000:
                    raise ValueError('SSH 串口响应过大')
        line, self.buffer = self.buffer.split(b'\n', 1)
        try:
            result = json.loads(line)
            if not result['ok']:
                raise ValueError('主控串口：' + result.get('error', '操作失败'))
            return result['value']
        except (json.JSONDecodeError, KeyError, TypeError) as error:
            raise ValueError('SSH 串口响应无效') from error

    def _request(self, operation, **kwargs):
        try:
            self._send_raw((json.dumps(dict(operation=operation, **kwargs)) + '\n').encode())
            return self._receive()
        except BaseException:
            # Never retry writes: the controller may already have committed them.
            self.close()
            raise

    def read_all(self):
        return {int(motor): row for motor, row in self._request('read_all').items()}

    def read(self, motor, name):
        return self._request('read', motor=motor, name=name)

    def write(self, motor, name, value):
        from calibration import IDS, WRITABLE
        if type(motor) is not int or motor not in IDS or name not in WRITABLE or type(value) is not int:
            raise ValueError('无效关节或禁止写入的寄存器')
        return self._request('write', motor=motor, name=name, value=value)

    def close(self):
        process, self.process = self.process, None
        if process is not None:
            if process.stdin:
                try:
                    process.stdin.close()  # EOF releases remote serial port.
                except OSError:
                    pass
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1)
            if process.stdout:
                process.stdout.close()
        if self.stderr:
            self.stderr.close()
            self.stderr = None
        self.buffer = b''
