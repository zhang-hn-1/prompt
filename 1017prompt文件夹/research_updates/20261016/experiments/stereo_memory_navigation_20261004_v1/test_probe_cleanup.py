from pathlib import Path
import pytest
from probe_cleanup import export_then_close,seal_existing,install_sigterm_cleanup

def test_export_failure_still_closes_runtime():
 class R:
  def __init__(self):self.closed=[]
  def close(self,return_code):self.closed.append(return_code)
 r=R()
 def bad():raise ValueError('export failed')
 with pytest.raises(ValueError):export_then_close(r,bad,1)
 assert r.closed==[1]

def test_partial_output_is_sealed_and_marked_incomplete(tmp_path):
 d=tmp_path/'run';d.mkdir();(d/'partial.txt').write_text('partial actual evidence')
 p=seal_existing(d,dict(forced_kill=True,complete=False))
 import json
 s=json.loads(p.read_text());assert not s['complete'];assert s['forced_kill'];assert str(d/'partial.txt') in s['artifact_sha256']

def test_sigterm_executes_finally_export(tmp_path):
 import subprocess,sys,os,time,signal
 ready=tmp_path/'ready';final=tmp_path/'final'
 code="from probe_cleanup import install_sigterm_cleanup; import time; from pathlib import Path\ninstall_sigterm_cleanup()\ntry:\n Path("+repr(str(ready))+").write_text('ready'); time.sleep(30)\nfinally:\n Path("+repr(str(final))+").write_text('partial exported')\n"
 p=subprocess.Popen([sys.executable,'-c',code],env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parent)))
 try:
  end=time.monotonic()+5
  while not ready.exists() and time.monotonic()<end:time.sleep(.01)
  assert ready.exists();p.terminate();p.wait(timeout=5);assert final.read_text()=='partial exported'
 finally:
  if p.poll() is None:p.kill();p.wait()
