import http.client
import itertools
import json
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock
from calibration import Session,DemoBus
import server
from server import handler

class HttpTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.session=Session(DemoBus,self.temp.name,True)
        self.server=ThreadingHTTPServer(('127.0.0.1',0),handler(self.session,0))
        self.port=self.server.server_port
        self.server.RequestHandlerClass=handler(self.session,self.port)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join();self.session.disconnect();self.temp.cleanup()
    def request(self,path,body=None,origin=True,host=None):
        conn=http.client.HTTPConnection('127.0.0.1',self.port)
        headers={'Host':host or f'127.0.0.1:{self.port}'}
        if origin:headers['Origin']=f'http://127.0.0.1:{self.port}'
        conn.request('GET' if body is None else 'POST',path,None if body is None else json.dumps(body),headers)
        res=conn.getresponse();status=res.status;data=res.read();conn.close();return status,data
    def test_disconnected_start_assets_and_origin(self):
        self.assertFalse(json.loads(self.request('/api/state')[1])['connected'])
        for path in ['/','/app.js','/i18n.js','/duck-view.js','/duck.bin']:self.assertEqual(self.request(path)[0],200)
        self.assertEqual(self.request('/api/connect',{},origin=False)[0],403)
        self.assertEqual(self.request('/api/state',host='evil.example')[0],403)
        self.assertFalse(self.session.bus)
    def test_full_demo_workflow_and_wrong_token(self):
        self.assertEqual(self.request('/api/connect',{})[0],200)
        for kind,value in [('zero',2100),('minimum',1700),('maximum',2500)]:
            self.assertEqual(self.request('/api/demo-position',dict(motor=20,position=value))[0],200)
            self.assertEqual(self.request('/api/capture',dict(motor=20,kind=kind))[0],200)
        status,data=self.request('/api/preview',dict(motor=20));self.assertEqual(status,200)
        token=json.loads(data)['preview']['token']
        self.assertEqual(self.request('/api/apply',dict(motor=20,token=token))[0],200)
        self.assertEqual(self.session.bus.rows[20]['offset'],-52)
        status,data=self.request('/api/apply',dict(motor=20,token=token))
        self.assertEqual(status,409);self.assertFalse(json.loads(data)['disconnected'])
        self.assertIsNotNone(self.session.bus)
    def test_rejected_request_keeps_session_and_captures(self):
        self.request('/api/connect',{})
        backup=self.session.backup
        self.request('/api/capture',dict(motor=20,kind='zero'))
        for body in [dict(motor=254,kind='zero'),dict(motor=20,kind='bogus')]:
            status,data=self.request('/api/capture',body)
            self.assertEqual(status,409);self.assertFalse(json.loads(data)['disconnected'])
        self.session.bus.rows[21]['position']=2048
        reads=itertools.cycle([2048,2100])
        with mock.patch.object(self.session.bus,'read',side_effect=lambda m,n:next(reads) if n=='position' and m==21 else self.session.bus.rows[m][n]):
            self.assertEqual(self.request('/api/capture',dict(motor=21,kind='zero'))[0],409)
        self.assertEqual(self.request('/api/connect',{})[0],409)
        self.assertIsNotNone(self.session.bus)
        self.assertEqual(self.session.backup,backup)
        self.assertEqual(self.session.marks,{20:{'zero':2048}})
    def test_hardware_fault_disconnects(self):
        self.request('/api/connect',{})
        self.session.bus.rows[20]['torque']=1
        self.assertEqual(self.request('/api/state')[0],409)
        self.assertIsNone(self.session.bus)

class AssetTests(unittest.TestCase):
    def test_missing_asset_fails_at_startup(self):
        with mock.patch.object(server,'MODEL',Path('/nonexistent/duck.bin')):
            with self.assertRaisesRegex(FileNotFoundError,'duck.bin'):handler(Session(DemoBus,'.',True),0)

if __name__=='__main__':unittest.main()
