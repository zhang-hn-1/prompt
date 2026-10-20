import subprocess,sys,time,json,os,signal
from pathlib import Path

def test_parent_termination_preserves_child_identity_and_terminal_status(tmp_path):
 module=Path(__file__).with_name('durable_stage_supervision_v1.py')
 parent=tmp_path/'parent.py';status=tmp_path/'status.json';log=tmp_path/'child.log'
 parent.write_text('import sys\nfrom pathlib import Path\nsys.path.insert(0,'+repr(str(module.parent))+')\nfrom durable_stage_supervision_v1 import run_durable\nrun_durable([sys.executable,"-c","import time; time.sleep(30)"],Path('+repr(str(status))+'),Path('+repr(str(log))+'),timeout=5,grace=1)\n')
 p=subprocess.Popen([sys.executable,str(parent)])
 try:
  deadline=time.monotonic()+5
  while p.poll() is None and time.monotonic()<deadline:
   if status.exists() and json.loads(status.read_text()).get('status')=='running':break
   time.sleep(.02)
  assert status.exists(), 'child identity must be saved before wait'
  initial=json.loads(status.read_text());assert initial['status']=='running' and initial['child_pid']>0
  p.terminate();p.wait(timeout=4);closed=json.loads(status.read_text())
  assert closed['status']=='interrupted' and closed['parent_signal']==signal.SIGTERM and closed['child_returncode'] is not None
  assert closed['child_pid']==initial['child_pid'] and closed['no_retry']
  try:os.kill(closed['child_pid'],0)
  except ProcessLookupError:pass
  else:raise AssertionError('child must be closed')
 finally:
  if p.poll() is None:p.kill();p.wait()

def test_normal_close_and_no_implicit_retry(tmp_path):
 import pytest
 from durable_stage_supervision_v1 import run_durable
 s=tmp_path/'status.json';r=run_durable([sys.executable,'-c','print("done")'],s,tmp_path/'log',timeout=2,grace=1)
 assert r['status']=='finished' and r['child_returncode']==0 and not r['timeout']
 before=s.read_bytes()
 with pytest.raises(FileExistsError):run_durable([sys.executable,'-c','print("retry")'],s,tmp_path/'log',timeout=2)
 assert s.read_bytes()==before

def test_timeout_is_not_success_and_child_is_closed(tmp_path):
 from durable_stage_supervision_v1 import run_durable
 r=run_durable([sys.executable,'-c','import time;time.sleep(30)'],tmp_path/'s.json',tmp_path/'l',timeout=.1,grace=1)
 assert r['status']=='timeout' and r['timeout'] and r['child_returncode'] is not None

def test_launch_error_is_terminal_and_signal_handlers_restored(tmp_path):
 import pytest
 from durable_stage_supervision_v1 import run_durable
 s=tmp_path/'status.json';old=signal.getsignal(signal.SIGTERM)
 with pytest.raises(FileNotFoundError):run_durable(['/tmp/no_such_spherical_robot_test_executable'],s,tmp_path/'l',timeout=1)
 r=json.loads(s.read_text());assert r['status']=='runtime_failed' and r['failure_type']=='FileNotFoundError'
 assert signal.getsignal(signal.SIGTERM)==old
