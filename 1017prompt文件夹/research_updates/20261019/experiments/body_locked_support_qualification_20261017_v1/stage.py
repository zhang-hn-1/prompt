from common import *
import os,sys,subprocess
cfg=load(D/'protocol.json')
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
for name in ['predict.py','replay.py','score.py']:
    child=subprocess.Popen([sys.executable,'-B',str(D/name)],env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
    put(D/(name[:-3]+'_process.json'),dict(pid=child.pid,parent_pid=os.getpid(),state='running'))
    rc=child.wait();put(D/(name[:-3]+'_process.json'),dict(pid=child.pid,parent_pid=os.getpid(),state='finished' if rc==0 else 'runtime_failed',returncode=rc));assert rc==0,(name,rc)
    if name=='predict.py':
        assert load(D/'prediction_seal.json')['complete']
        for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
        put(D/'postprediction_input_identity_guard.json',dict(utc=now(),inputs_exact=len(cfg['input_sha256']),teacher_numeric_arrays_loaded=False,prediction_seal_sha256=sha(D/'prediction_seal.json')))

