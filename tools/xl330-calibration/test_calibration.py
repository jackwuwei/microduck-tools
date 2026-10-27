import json
from pathlib import Path
import tempfile
import unittest
from calibration import DemoBus, Session, plan, CONFIG, IDS, SerialBus


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.bus=DemoBus()
        self.session=Session(lambda:self.bus,self.temp.name,True)
        self.session.connect()
    def tearDown(self):
        self.session.disconnect()
        self.temp.cleanup()
    def capture(self,kind,value):
        self.bus.rows[20]['position']=value
        self.session.capture(20,kind)
    def stage(self):
        self.capture('zero',2100)
        self.capture('minimum',1700)
        self.capture('maximum',2500)
    def test_offset_and_limits_use_existing_offset(self):
        result=plan({'offset':100,'drive':5},{'zero':2100,'minimum':1700,'maximum':2500})
        self.assertEqual(result,dict(drive=5,offset=48,minimum=1648,maximum=2448))
    def test_reject_wrap_wrong_order_and_offset_overflow(self):
        for marks in [dict(zero=0,minimum=0,maximum=3000),dict(zero=2048,minimum=3000,maximum=1000),
                      dict(zero=2048,minimum=2048,maximum=2500),dict(zero=4096+2048,minimum=5000,maximum=7000)]:
            with self.assertRaises(ValueError): plan({'offset':0,'drive':0},marks)
    def test_apply_persists_and_restore_uses_original(self):
        self.stage()
        preview=self.session.prepare(20)
        self.session.apply(20,preview['token'])
        self.assertEqual(self.bus.rows[20]['offset'],-52)
        self.assertEqual(self.bus.rows[20]['minimum'],1648)
        self.assertIn(20,self.session.completed)
        self.session.restore(20)
        self.assertEqual({k:self.bus.rows[20][k] for k in CONFIG},{k:self.session.original[20][k] for k in CONFIG})
        self.assertNotIn(20,self.session.completed)
    def test_preview_is_single_use_and_capture_invalidates(self):
        self.stage()
        token=self.session.prepare(20)['token']
        self.capture('maximum',2550)
        with self.assertRaises(ValueError): self.session.apply(20,token)
        token=self.session.prepare(20)['token']
        self.session.apply(20,token)
        with self.assertRaises(ValueError): self.session.apply(20,token)
    def test_direction_preserves_other_bits_and_clears_marks(self):
        self.bus.rows[20]['drive']=4
        self.session.expected[20]['drive']=4
        self.stage()
        self.session.direction(20,True)
        self.assertEqual(self.bus.rows[20]['drive'],5)
        self.assertNotIn(20,self.session.marks)
    def test_backup_exists_before_first_write_and_can_restore_after_reconnect(self):
        path=Path(self.session.backup)
        self.assertEqual(json.loads(path.read_text())['joints']['20']['offset'],0)
        self.stage()
        self.session.apply(20,self.session.prepare(20)['token'])
        self.session.disconnect()
        self.session.connect()
        self.session.restore_file(20,path.name)
        self.assertEqual(self.bus.rows[20]['offset'],0)
    def test_backup_rejects_escape_and_demo_mismatch(self):
        with self.assertRaises(ValueError): self.session.restore_file(20,'../outside.json')
        path=Path(self.session.backup)
        data=json.loads(path.read_text());data['demo']=False
        path.write_text(json.dumps(data))
        with self.assertRaises(ValueError): self.session.restore_file(20,path.name)
    def test_safety_interlocks_and_external_changes(self):
        for key,value in [('torque',1),('error',4),('model',1190),('mode',4),('voltage',70),('temperature',65),('offset',10)]:
            old=self.bus.rows[21][key]
            self.bus.rows[21][key]=value
            with self.assertRaises(ValueError): self.session.prepare(20)
            self.bus.rows[21][key]=old
    def test_unstable_capture_is_rejected(self):
        original=self.bus.read
        count=0
        def changing(motor,name):
            nonlocal count
            if motor==20 and name=='position':
                count+=1
                return 2000+count*10
            return original(motor,name)
        self.bus.read=changing
        with self.assertRaisesRegex(ValueError,'静止'): self.session.capture(20,'zero')
        self.assertNotIn(20,self.session.marks)
    def test_never_writes_motion_or_torque(self):
        self.stage()
        writes=[]
        original=self.bus.write
        def record(motor,name,value):
            writes.append(name);original(motor,name,value)
        self.bus.write=record
        self.session.apply(20,self.session.prepare(20)['token'])
        self.session.restore(20)
        self.assertTrue(writes)
        self.assertTrue(set(writes)<=set(CONFIG))
        self.assertTrue(all(self.bus.rows[i]['torque']==0 for i in IDS))
    def test_partial_write_never_marks_complete(self):
        self.stage()
        def fail(*args): raise ValueError('模拟通信中断')
        self.bus.write=fail
        with self.assertRaises(ValueError): self.session.apply(20,self.session.prepare(20)['token'])
        self.assertNotIn(20,self.session.completed)
        self.assertIsNone(self.session.preview)

    def test_microduck_2s_checks(self):
        self.session.microduck_2s=True
        rows=self.bus.rows
        for row in rows.values(): row.update(voltage=80,error=1)
        self.session.guard(rows)
        for key,value in [('error',5),('error',16),('torque',1),('temperature',60),('voltage',83),('voltage',65)]:
            old=rows[20][key];rows[20][key]=value
            with self.assertRaises(ValueError): self.session.guard(rows)
            rows[20][key]=old
        self.session.microduck_2s=False
        with self.assertRaises(ValueError): self.session.guard(rows)

    def test_transport_signed_offset_and_register_allowlist(self):
        from types import SimpleNamespace
        bus=SerialBus.__new__(SerialBus)
        bus.port=SimpleNamespace(ser=SimpleNamespace(in_waiting=0))
        calls=[]
        class Packet:
            def write4ByteTxRx(self,port,motor,address,value):
                calls.append((motor,address,value));return 0,0
            def read4ByteTxRx(self,port,motor,address):
                return 0xffffffcc,0,0
        bus.packet=Packet()
        bus.write(20,'offset',-52)
        self.assertEqual(calls,[(20,20,0xffffffcc)])
        self.assertEqual(bus.read(20,'offset'),-52)
        with self.assertRaises(ValueError):bus.write(20,'torque',1)
        with self.assertRaises(ValueError):bus.write(20,'position',2048)
        bus.port.ser.in_waiting=1
        with self.assertRaises(ValueError):bus.read(20,'offset')

if __name__=='__main__':unittest.main()
