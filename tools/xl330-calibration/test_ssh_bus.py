"""Exercise the real stdio protocol against a local fake serial worker."""
import inspect
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from calibration import Session
from server import connection_args
from ssh_bus import SshBus, remote_worker


class FakeSshBus(SshBus):
    def _program(self):
        source = Path(__file__).with_name('calibration.py').read_text()
        source += '''
class SerialBus(DemoBus):
    def __init__(self, *args):
        super().__init__()
        for row in self.rows.values(): row['voltage'] = 79
'''
        return source + '\n' + inspect.getsource(remote_worker) + "\nremote_worker({'serial':'fake','baud':1000000,'microduck_2s':True,'pythonpath':[]})\n"


class SshTests(unittest.TestCase):
    def setUp(self):
        real_popen = subprocess.Popen
        self.commands = []
        def local_worker(command, **kwargs):
            self.commands.append(command)
            return real_popen([sys.executable, '-u', '-c', SshBus.BOOTSTRAP], **kwargs)
        self.patch = patch('ssh_bus.subprocess.Popen', side_effect=local_worker)
        self.patch.start()
        self.bus = FakeSshBus(timeout=2)
    def tearDown(self):
        self.bus.close()
        self.patch.stop()

    def test_remote_reads_write_readback_and_cleanup(self):
        self.bus.open()
        self.assertEqual(self.bus.read(20, 'voltage'), 79)
        self.assertEqual(len(self.bus.read_all()), 15)
        self.bus.write(20, 'offset', -52)
        self.assertEqual(self.bus.read(20, 'offset'), -52)
        process = self.bus.process
        self.assertEqual(self.commands[0][0], 'ssh')
        self.assertIn('BatchMode=yes', self.commands[0])
        self.assertIn('microduck', self.commands[0])
        self.bus.close()
        self.assertIsNotNone(process.poll())

    def test_session_keeps_backup_on_computer(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Session(lambda:self.bus, directory, microduck_2s=True)
            try:
                session.connect()
                self.assertEqual(session.state()['joints'][0]['telemetry']['voltage'], 79)
                self.assertEqual(Path(session.backup).parent, Path(directory))
                self.assertTrue(Path(session.backup).is_file())
            finally:
                session.disconnect()

    def test_worker_rejects_forbidden_write_and_closes(self):
        self.bus.open()
        with self.assertRaisesRegex(ValueError, '禁止写入'):
            self.bus._request('write', motor=20, name='torque', value=1)
        self.assertIsNone(self.bus.process)

    def test_transport_failure_is_not_retried(self):
        self.bus.open()
        self.bus.process.kill()
        self.bus.process.wait()
        with self.assertRaises(ValueError):
            self.bus.write(20, 'offset', 0)
        self.assertIsNone(self.bus.process)
        self.assertEqual(len(self.commands), 1)

    def test_response_timeout_closes_process(self):
        self.bus.timeout = .05
        self.bus._program = lambda:'import time; time.sleep(10)'
        with self.assertRaisesRegex(ValueError, '超时'):
            self.bus.open()
        self.assertIsNone(self.bus.process)

    def test_host_cannot_be_ssh_option_or_shell_command(self):
        for host in ('-oProxyCommand=bad', 'microduck; touch /tmp/no', '$(id)'):
            with self.assertRaises(ValueError): SshBus(host)

    def test_custom_interpreter_is_shell_quoted(self):
        self.bus.remote_python = '/opt/remote venv/bin/python'
        self.bus.open()
        self.assertTrue(self.commands[0][-1].startswith("'/opt/remote venv/bin/python' -u -c "))

    def test_remote_dependency_paths_are_configurable(self):
        bus = SshBus(pythonpath=['~/custom deps', '/opt/serial/lib'])
        program = bus._program()
        self.assertIn("'pythonpath': ['~/custom deps', '/opt/serial/lib']", program)
        self.assertNotIn('/home/microduck/diagnostics/readlibs', program)
        args = connection_args(['--remote-python', '/opt/python3',
                                '--remote-pythonpath', '~/first',
                                '--remote-pythonpath', '/opt/second'])
        self.assertEqual(args.remote_python, '/opt/python3')
        self.assertEqual(args.remote_pythonpath, ['~/first', '/opt/second'])

    def test_cli_default_ssh_and_explicit_local_serial(self):
        default = connection_args([])
        self.assertEqual(default.ssh, 'microduck')
        self.assertEqual(default.remote_serial, '/dev/ttyS2')
        self.assertTrue(default.microduck_2s)
        self.assertFalse(connection_args(['--demo']).microduck_2s)
        local = connection_args(['--serial', '/dev/cu.test'])
        self.assertIsNone(local.ssh)
        self.assertEqual(local.serial, '/dev/cu.test')
        with self.assertRaises(SystemExit), patch('sys.stderr'):
            connection_args(['--ssh', 'microduck', '--serial', '/dev/cu.test'])


if __name__ == '__main__': unittest.main()
