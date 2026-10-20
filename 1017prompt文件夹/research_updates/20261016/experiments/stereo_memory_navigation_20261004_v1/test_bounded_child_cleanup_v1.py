import sys
from bounded_child_cleanup_v1 import run_bounded

def test_sigterm_cleanup(tmp_path):
 flag=tmp_path/'cleanup'
 code="import signal,time,pathlib;signal.signal(signal.SIGTERM,lambda *a:(pathlib.Path("+repr(str(flag))+").write_text('closed'),exit(0)));print('ready',flush=True);time.sleep(10)"
 with (tmp_path/'log').open('w') as log:r=run_bounded([sys.executable,'-c',code],log,timeout=.3,grace=1)
 assert r['timeout'] and not r['forced_kill'] and r['returncode']==0 and flag.read_text()=='closed'

def test_ignoring_sigterm_is_forced(tmp_path):
 code='import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);print("ready",flush=True);time.sleep(10)'
 with (tmp_path/'log').open('w') as log:r=run_bounded([sys.executable,'-c',code],log,timeout=.3,grace=.1)
 assert r['timeout'] and r['forced_kill'] and r['returncode']!=0
