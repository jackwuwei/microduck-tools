"""XL330 commissioning: torque-off manual capture; no motion or torque-on commands."""
from __future__ import annotations
import json
import os
from pathlib import Path
import secrets
import time

JOINTS = [(20,'左腿 · 髋部转向'),(21,'左腿 · 髋部侧摆'),(22,'左腿 · 髋部前后'),(23,'左腿 · 膝盖'),(24,'左腿 · 脚踝'),
          (30,'头颈 · 颈部俯仰'),(31,'头颈 · 头部俯仰'),(32,'头颈 · 转向'),(33,'头颈 · 侧摆'),(34,'鸭嘴'),
          (10,'右腿 · 髋部转向'),(11,'右腿 · 髋部侧摆'),(12,'右腿 · 髋部前后'),(13,'右腿 · 膝盖'),(14,'右腿 · 脚踝')]
IDS = tuple(i for i, _ in JOINTS)
# address, byte width, signed
REGISTERS = {'model':(0,2,False),'firmware':(6,1,False),'drive':(10,1,False),'mode':(11,1,False),
             'offset':(20,4,True),'maximum':(48,4,False),'minimum':(52,4,False),
             'torque':(64,1,False),'error':(70,1,False),'position':(132,4,True),
             'voltage':(144,2,False),'temperature':(146,1,False)}
CONFIG = ('drive','offset','minimum','maximum')
WRITABLE = frozenset(CONFIG)


def plan(row, marks):
    """No modulo: an ambiguous wrap/multiple turn must be recaptured, never guessed."""
    if set(marks) != {'zero','minimum','maximum'}:
        raise ValueError('先采集零位和两个限位')
    absolute = marks['zero'] - row['offset']
    if not 0 <= absolute <= 4095:
        raise ValueError('编码器处于跨圈坐标；请断开工具，正常断电重启舵机后重连并重新采集')
    offset = row['offset'] + 2048 - marks['zero']
    if not -1024 <= offset <= 1024:
        raise ValueError('零位需要的偏移超出 ±1024；请调整舵盘安装位置后重新采集')
    delta = offset - row['offset']
    lo, hi = marks['minimum'] + delta, marks['maximum'] + delta
    if not 0 <= lo < 2048 < hi <= 4095:
        raise ValueError('限位必须位于 0–4095 且包围零位；请按模型负向/正向重新采集，勿跨整圈')
    return {'drive':row['drive'], 'offset':offset, 'minimum':lo, 'maximum':hi}


class SerialBus:
    def __init__(self, port, baud, microduck_2s=False):
        from dynamixel_sdk import PacketHandler, PortHandler
        self.port = PortHandler(port)
        self.packet = PacketHandler(2.0)
        self.baud = baud
        self.microduck_2s = microduck_2s

    def open(self):
        import fcntl
        import termios
        try:
            if not self.port.setBaudRate(self.baud):
                raise ValueError('无法打开串口')
            self.port.ser.exclusive = True
            fcntl.ioctl(self.port.ser.fileno(), termios.TIOCEXCL)
            self.port.ser.timeout = .01
            end = time.monotonic() + 2
            while time.monotonic() < end:
                if self.port.ser.read(max(1,self.port.ser.in_waiting)):
                    raise ValueError('总线已有数据；请先停止 robotd、观察台和其他串口程序')
        except BaseException:
            self.close()
            raise

    def close(self):
        self.port.closePort()

    def check(self, result, error):
        # Alert bit carries no instruction error; inspect Hardware Error Status in guard.
        if getattr(self, "microduck_2s", False) and error == 0x80:
            error = 0
        if result != 0 or error:
            raise ValueError(f'DYNAMIXEL 通信失败：{self.packet.getTxRxResult(result)} / {self.packet.getRxPacketError(error)}')

    def read(self, motor, name):
        address,size,signed = REGISTERS[name]
        if self.port.ser.in_waiting:
            raise ValueError('发现其他总线流量，停止会话')
        value,result,error = getattr(self.packet,f'read{size}ByteTxRx')(self.port,motor,address)
        self.check(result,error)
        return value - (1 << (size*8)) if signed and value & (1 << (size*8-1)) else value

    def write(self, motor, name, value):
        if name not in WRITABLE:
            raise ValueError('该寄存器禁止写入')
        address,size,_ = REGISTERS[name]
        result,error = getattr(self.packet,f'write{size}ByteTxRx')(self.port,motor,address,value & ((1 << (size*8))-1))
        self.check(result,error)
        time.sleep(.025)
        if self.read(motor,name) != value:
            raise ValueError(f'ID {motor} {name} 回读不一致')


class DemoBus:
    def __init__(self):
        self.rows = {i:dict(model=1200,firmware=46,drive=0,mode=3,offset=0,minimum=0,maximum=4095,
                            torque=0,error=0,position=2048,voltage=50,temperature=25) for i in IDS}
    def open(self): pass
    def close(self): pass
    def read(self,motor,name): return self.rows[motor][name]
    def write(self,motor,name,value):
        row = self.rows[motor]
        if name == 'offset': row['position'] += value-row['offset']
        row[name] = value


class Session:
    def __init__(self, factory, backup_dir, demo=False, microduck_2s=False):
        self.factory,self.backup_dir,self.demo = factory,Path(backup_dir),demo
        self.microduck_2s = microduck_2s
        self.bus = None
        self.original = {}
        self.expected = {}
        self.marks = {}
        self.preview = None
        self.backup = None
        self.error = None
        self.completed = set()

    def read_all(self):
        if self.bus is None: raise ValueError('请先连接')
        if hasattr(self.bus, 'read_all'):
            return self.bus.read_all()
        return {i:{name:self.bus.read(i,name) for name in REGISTERS} for i in IDS}

    def guard(self, rows):
        for i,row in rows.items():
            if row['model'] != 1200 or row['mode'] != 3:
                raise ValueError(f'ID {i}：仅支持 XL330-M288、位置模式 3；不自动切换模式')
            allowed_error = 1 if self.microduck_2s else 0
            if row['torque'] or row['error'] & ~allowed_error:
                raise ValueError(f'ID {i}：必须扭矩关闭、硬件错误为零')
            voltage_ok = 66 <= row['voltage'] <= 82 if self.microduck_2s else 37 <= row['voltage'] <= 60
            if not voltage_ok or row['temperature'] >= 60:
                raise ValueError(f'ID {i}：电压或温度不符合标定条件')

    def checked(self):
        rows = self.read_all()
        self.guard(rows)
        for i,row in rows.items():
            if any(row[k] != self.expected[i][k] for k in CONFIG):
                raise ValueError(f'ID {i} 参数被其他程序修改；请断开后重新连接')
        return rows

    def connect(self):
        if self.bus: raise ValueError('已经连接')
        self.bus = self.factory()
        try:
            self.bus.open()
            rows = self.read_all()
            self.guard(rows)
            self.backup_dir.mkdir(parents=True,exist_ok=True)
            path = self.backup_dir / (time.strftime('%Y%m%d-%H%M%S')+'-'+secrets.token_hex(4)+'.json')
            payload = dict(version=1,demo=self.demo,created_at=time.time(),joints=rows)
            with path.open('x',encoding='utf-8') as file:
                json.dump(payload,file,ensure_ascii=False,indent=2)
                file.flush()
                os.fsync(file.fileno())
            self.backup = str(path)
            self.original = rows
            self.expected = {i:{k:r[k] for k in CONFIG} for i,r in rows.items()}
            self.marks = {}
            self.completed = set()
            self.preview = None
            self.error = None
        except BaseException:
            self.disconnect()
            raise

    def disconnect(self):
        if self.bus: self.bus.close()
        self.bus = None
        self.preview = None

    def capture(self,motor,kind):
        if kind not in ('zero','minimum','maximum'): raise ValueError('未知采集类型')
        rows = self.checked()
        # Ten actual responses over ~0.5 s, rejecting a moving hand or a wrap.
        samples = []
        for _ in range(10):
            samples.append(self.bus.read(motor,'position'))
            if not self.demo: time.sleep(.05)
        if max(samples)-min(samples) > 8: raise ValueError('关节尚未静止，请稳定后重试')
        self.checked()
        self.marks.setdefault(motor,{})[kind] = round(sum(samples)/len(samples))
        self.preview = None
        self.completed.discard(motor)

    def direction(self,motor,reverse):
        if type(reverse) is not bool: raise ValueError('方向必须为布尔值')
        self.checked()
        drive = (self.expected[motor]['drive'] & ~1) | int(reverse)
        if drive != self.expected[motor]['drive']:
            self.bus.write(motor,'drive',drive)
            self.expected[motor]['drive'] = drive
            self.marks.pop(motor,None)
            self.completed.discard(motor)
        self.preview = None

    def prepare(self,motor):
        rows = self.checked()
        values = plan(rows[motor],self.marks.get(motor,{}))
        self.preview = dict(token=secrets.token_hex(16),motor=motor,before=self.expected[motor].copy(),after=values)
        return self.preview

    def write_config(self,motor,values):
        # Temporarily widen while torque is OFF to avoid min/max ordering conflicts.
        for name,value in [('minimum',0),('maximum',4095),('drive',values['drive']),
                           ('offset',values['offset']),('minimum',values['minimum']),('maximum',values['maximum'])]:
            if self.bus.read(motor,name) != value:
                self.bus.write(motor,name,value)
        for name,value in values.items():
            if self.bus.read(motor,name) != value: raise ValueError('最终参数回读失败')
        self.expected[motor] = values.copy()

    def apply(self,motor,token):
        self.checked()
        preview = self.preview
        if not preview or preview['motor'] != motor or token != preview['token']:
            raise ValueError('预览已过期，请重新预览')
        self.preview = None  # single-use, including failed writes
        self.write_config(motor,preview['after'])
        self.marks.pop(motor,None)
        self.completed.add(motor)

    def restore(self,motor):
        self.checked()
        values = {k:self.original[motor][k] for k in CONFIG}
        self.write_config(motor,values)
        self.marks.pop(motor,None)
        self.completed.discard(motor)
        self.preview = None

    def restore_file(self,motor,path):
        # Only server-created backups under the configured directory are eligible.
        candidate = (self.backup_dir / path).resolve()
        if candidate.parent != self.backup_dir.resolve(): raise ValueError('无效备份路径')
        data = json.loads(candidate.read_text())
        if data.get('version') != 1 or data.get('demo') is not self.demo: raise ValueError('备份版本或模式不匹配')
        row = data['joints'][str(motor)]
        if row['model'] != 1200 or row['mode'] != 3: raise ValueError('备份型号或模式不匹配')
        values = {k:row[k] for k in CONFIG}
        if any(type(v) is not int for v in values.values()) or not (0 <= values['drive'] <= 13 and -1024 <= values['offset'] <= 1024 and 0 <= values['minimum'] < values['maximum'] <= 4095):
            raise ValueError('备份寄存器值无效')
        self.checked()
        self.preview = None
        self.write_config(motor,values)
        self.marks.pop(motor,None)
        self.completed.discard(motor)

    def state(self):
        rows = self.checked() if self.bus else {}
        return dict(connection=getattr(self,'connection',None),demo=self.demo,connected=self.bus is not None,error=self.error,backup=self.backup,
                    joints=[dict(id=i,label=label,telemetry=rows.get(i),marks=self.marks.get(i,{}),complete=i in self.completed) for i,label in JOINTS],
                    preview=self.preview)
