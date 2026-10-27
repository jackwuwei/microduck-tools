#!/usr/bin/env python3
"""Local XL330 calibration UI; default transport is SSH to microduck."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from urllib.parse import urlsplit
from calibration import Session, SerialBus, DemoBus, IDS
from ssh_bus import SshBus

FOLDER = Path(__file__).resolve().parent
ROOT = FOLDER.parents[1]
MODEL = ROOT/'assets/models/duck-web-mouth-v2.bin'


def handler(session, port):
    lock = threading.Lock()
    files = {'/':(FOLDER/'index.html','text/html; charset=utf-8'),
             '/app.js':(FOLDER/'app.js','text/javascript; charset=utf-8'),
             '/i18n.js':(FOLDER/'i18n.js','text/javascript; charset=utf-8'),
             '/duck-view.js':(FOLDER/'duck-view.js','text/javascript; charset=utf-8'),
             '/duck.bin':(MODEL,'application/octet-stream')}
    hosts = {f'127.0.0.1:{port}',f'localhost:{port}'}
    class Handler(BaseHTTPRequestHandler):
        def reply(self,status,data,mime='application/json; charset=utf-8'):
            body = data if isinstance(data,bytes) else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type',mime)
            self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'")
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError): pass
        def do_GET(self):
            if self.headers.get('Host') not in hosts: return self.reply(403,{'error':'仅允许本机访问'})
            path = urlsplit(self.path).path
            if path in files:
                file,mime = files[path]
                return self.reply(200,file.read_bytes(),mime)
            with lock:
                try:
                    if path == '/api/state': return self.reply(200,session.state())
                    if path == '/api/backups': return self.reply(200,sorted(p.name for p in session.backup_dir.glob('*.json')))
                except Exception as error:
                    session.error = str(error)
                    session.disconnect()
                    return self.reply(409,{'error':str(error),'disconnected':True})
            self.reply(404,{'error':'未找到'})
        def setup(self):
            super().setup()
            self.connection.settimeout(10)
        def do_POST(self):
            host = self.headers.get('Host')
            if host not in hosts or self.headers.get('Origin') != f'http://{host}':
                return self.reply(403,{'error':'需要同源请求'})
            with lock:
                try:
                    length = int(self.headers.get('Content-Length','0'))
                    if not 0 < length <= 2048: raise ValueError('请求大小无效')
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data,dict): raise ValueError('请求格式无效')
                    action = urlsplit(self.path).path
                    if action == '/api/connect': session.connect()
                    elif action == '/api/disconnect': session.disconnect()
                    else:
                        motor = data.get('motor')
                        if type(motor) is not int or motor not in IDS: raise ValueError('无效关节 ID')
                        if action == '/api/capture': session.capture(motor,data.get('kind'))
                        elif action == '/api/direction': session.direction(motor,data.get('reverse'))
                        elif action == '/api/preview': session.prepare(motor)
                        elif action == '/api/apply': session.apply(motor,data.get('token'))
                        elif action == '/api/restore': session.restore(motor)
                        elif action == '/api/restore-file': session.restore_file(motor,data.get('file',''))
                        elif action == '/api/demo-position' and session.demo:
                            value = data.get('position')
                            if type(value) is not int or not 0 <= value <= 4095: raise ValueError('无效模拟位置')
                            session.bus.rows[motor]['position'] = value
                        else: raise ValueError('未知操作')
                    return self.reply(200,session.state())
                except Exception as error:
                    # A failed EEPROM operation may be partially committed. Close the
                    # session; reconnect creates a new backup before any further write.
                    session.error = str(error)
                    session.disconnect()
                    return self.reply(409,{'error':str(error),'disconnected':True})
        def log_message(self,*_args): pass
    return Handler


def connection_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo',action='store_true')
    parser.add_argument('--microduck-2s',action='store_true',default=None,help='本项目真机默认启用 Microduck 官方 2S 供电配置；保留电压报警，仅允许 6.6–8.2 V')
    transport = parser.add_mutually_exclusive_group()
    transport.add_argument('--ssh',metavar='HOST',help='SSH 目标；不指定连接方式时默认为 microduck')
    transport.add_argument('--serial',help='电脑本地串口，例如 /dev/cu.usbserial-...')
    parser.add_argument('--remote-serial',default='/dev/ttyS2',help='SSH 主控串口，默认 /dev/ttyS2')
    parser.add_argument('--remote-python',default='python3',help='主控 Python 解释器命令或绝对路径')
    parser.add_argument('--remote-pythonpath',action='append',metavar='PATH',help='主控依赖目录，可重复；指定后覆盖默认搜索目录，支持 ~')
    parser.add_argument('--baud',type=int,default=1_000_000)
    parser.add_argument('--port',type=int,default=18092)
    parser.add_argument('--backup-dir',type=Path,default=ROOT/'work/xl330-calibration-backups')
    args = parser.parse_args(argv)
    if args.microduck_2s is None: args.microduck_2s = not args.demo
    if args.demo and (args.ssh or args.serial): parser.error('离线模式不能指定真机连接')
    if not args.demo and not args.serial: args.ssh = args.ssh or 'microduck'
    if not 1024 <= args.port <= 65535 or args.baud <= 0: parser.error('端口或波特率无效')
    return args


def main():
    args = connection_args()
    if args.demo:
        factory = DemoBus
    elif args.ssh:
        factory = lambda:SshBus(args.ssh,args.remote_serial,args.baud,args.microduck_2s,
                              remote_python=args.remote_python,pythonpath=args.remote_pythonpath)
    else:
        factory = lambda:SerialBus(args.serial,args.baud,args.microduck_2s)
    session = Session(factory,args.backup_dir,args.demo,args.microduck_2s)
    session.connection = f"SSH {args.ssh} → {args.remote_serial}" if args.ssh else (f"电脑串口 {args.serial}" if args.serial else None)
    server = ThreadingHTTPServer(('127.0.0.1',args.port),handler(session,args.port))
    server.daemon_threads = True
    print(f'XL330 {"演示" if args.demo else "真机"}标定：http://127.0.0.1:{args.port}/ （未连接）',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        server.shutdown()
        session.disconnect()
        server.server_close()

if __name__ == '__main__': main()
