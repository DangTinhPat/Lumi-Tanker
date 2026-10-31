"""Host tests only: no real ESP connection or firmware upload."""
import hashlib
import importlib.util
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('ota_upload',Path(__file__).parents[1]/'tools/ota_upload.py')
ota=importlib.util.module_from_spec(spec);spec.loader.exec_module(ota)
MAC='70:04:1d:a8:ca:fc'


def state(ready=False, reboot=False):
    return {'HCR':{'armed':'0','enabled':'0','report_age_ms':'10'},
            'NET':{'mac':MAC},
            'OTA':{'supported':'1','maintenance':str(int(ready)),
                   'off_confirmed':str(int(ready)),'ready':str(int(ready)),
                   'elf_sha256':'new' if reboot else 'old'}}


class Console:
    def __init__(self, states):
        self.states=iter(states);self.commands=[]
    def request(self, command='STATUS'):
        self.commands.append(command)
        return next(self.states)


class OtaTests(unittest.TestCase):
    def test_upload_only_after_off_and_verify_reboot(self):
        console=Console([state(),state(),state(True),state(reboot=True)])
        calls=[]
        def upload():
            calls.append('upload')
            self.assertEqual(console.commands,['STATUS','OTA_BEGIN','STATUS'])
            return 0
        result=ota.update(console,MAC,{'elf_sha256':'new'},upload)
        self.assertEqual(calls,['upload'])
        self.assertEqual(result['OTA']['elf_sha256'],'new')
        self.assertNotIn('CLEAR_FAULT',console.commands)

    def test_unsupported_wrong_board_and_active_refuse_before_maintenance(self):
        cases=[state(),state(),state()]
        cases[0]['OTA']['supported']='0'
        cases[1]['NET']['mac']='00:00:00:00:00:00'
        cases[2]['HCR']['armed']='1'
        for s in cases:
            console=Console([s])
            with self.assertRaises(RuntimeError):
                ota.update(console,MAC,{},lambda:self.fail('must not upload'))
            self.assertEqual(console.commands,['STATUS'])

    def test_missing_off_confirmation_never_uploads(self):
        console=Console([state(),state()])
        with self.assertRaisesRegex(RuntimeError,'not ready'):
            ota.update(console,MAC,{},lambda:self.fail('must not upload'),wait_seconds=0)

    def test_transfer_failure_does_not_reboot_or_clear_fault(self):
        console=Console([state(),state(),state(True)])
        with self.assertRaisesRegex(RuntimeError,'transfer failed'):
            ota.update(console,MAC,{},lambda:1)
        self.assertEqual(console.commands,['STATUS','OTA_BEGIN','STATUS'])

    def test_transfer_success_requires_new_running_image(self):
        console=Console([state(),state(),state(True)])
        with self.assertRaisesRegex(RuntimeError,'reboot verification failed'):
            ota.update(console,MAC,{'elf_sha256':'new'},lambda:0,reboot_seconds=0)

    def test_freshness_and_every_ready_gate(self):
        for group,key,value in [('OTA','off_confirmed','0'),('OTA','ready','0'),
                                ('OTA','maintenance','0'),('HCR','enabled','1'),
                                ('HCR','report_age_ms','251')]:
            s=state(True);s[group][key]=value;self.assertFalse(ota.is_ready(s))

    def test_image_manifest_and_credentials(self):
        with tempfile.TemporaryDirectory() as directory:
            binary=Path(directory)/'firmware.bin'
            data=bytearray(300);data[0]=0xe9;struct.pack_into('<H',data,12,9)
            struct.pack_into('<I',data,32,0xabcd5432);data[176:208]=bytes(range(32))
            binary.write_bytes(data)
            credentials={'password':'test-only-password'}
            manifest={'environment':ota.ENV,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),
                      'elf_sha256':data[176:208].hex(),
                      'ota_auth_hash':hashlib.md5(credentials['password'].encode()).hexdigest()}
            binary.with_suffix('.ota.json').write_text(json.dumps(manifest))
            self.assertEqual(ota.validate_image(binary,credentials),manifest)
            with self.assertRaises(ValueError):ota.validate_image(binary,{'password':'wrong'})
            data[-1]=1;binary.write_bytes(data)
            with self.assertRaises(ValueError):ota.validate_image(binary,credentials)
            manifest['sha256']=hashlib.sha256(data).hexdigest();manifest['environment']='esp32-s3-other'
            binary.with_suffix('.ota.json').write_text(json.dumps(manifest))
            with self.assertRaises(ValueError):ota.validate_image(binary,credentials)

    def test_fragmented_console_lines(self):
        class Socket:
            def __enter__(self):return self
            def __exit__(self,*args):pass
            def settimeout(self,value):pass
            def sendall(self,value):self.sent=value
            chunks=iter([b'HCR armed=0 enab',b'led=0\nOTA supported=1 ready=0\nNE',
                         b'T mac=70:04:1d:a8:ca:fc\n'])
            def recv(self,size):return next(self.chunks)
        sock=Socket()
        with patch.object(ota.socket,'create_connection',return_value=sock):
            result=ota.Console('test').request()
        self.assertEqual(sock.sent,b'STATUS\n')
        self.assertEqual(result['HCR']['enabled'],'0')
        self.assertEqual(result['NET']['mac'],MAC)


if __name__=='__main__':unittest.main()
