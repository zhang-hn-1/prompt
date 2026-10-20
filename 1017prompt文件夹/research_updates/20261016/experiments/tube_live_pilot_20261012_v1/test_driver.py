import json,hashlib,pytest
from pathlib import Path
from driver import validate_admission

def test_missing_admission(tmp_path):
 p=tmp_path/'a';p.write_text('{}')
 with pytest.raises(ValueError):validate_admission(p)
def test_exact_driver_admission(tmp_path):
 p=tmp_path/'a';src=Path(__file__).with_name('driver.py');p.write_text(json.dumps(dict(launch_admitted=True,physical_episodes_max=1,retries=0,source_sha256={str(src.resolve()):hashlib.sha256(src.read_bytes()).hexdigest()})))
 assert validate_admission(p)['retries']==0
def test_source_changed(tmp_path):
 p=tmp_path/'a';src=Path(__file__).with_name('driver.py');p.write_text(json.dumps(dict(launch_admitted=True,physical_episodes_max=1,retries=0,source_sha256={str(src.resolve()):'0'*64})))
 with pytest.raises(ValueError):validate_admission(p)

def test_one_reservation(tmp_path):
 from driver import reserve_attempt
 p=tmp_path/'admission.json';p.write_text('{}')
 reserve_attempt(p)
 with pytest.raises(FileExistsError):reserve_attempt(p)

def test_cleanup_even_export_failure():
 from driver import export_close_seal
 called=[]
 class Runtime:
  def close(self,return_code):called.append('close')
 def bad():raise RuntimeError('export')
 with pytest.raises(RuntimeError):export_close_seal(Runtime(),bad,lambda:called.append('seal'),1)
 assert called==['close','seal']
def test_seal_even_close_failure():
 from driver import export_close_seal
 called=[]
 class Runtime:
  def close(self,return_code):raise RuntimeError('close')
 with pytest.raises(RuntimeError):export_close_seal(Runtime(),lambda:None,lambda:called.append('seal'),1)
 assert called==['seal']
