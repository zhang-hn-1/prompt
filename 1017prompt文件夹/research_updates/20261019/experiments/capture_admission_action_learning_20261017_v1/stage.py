from common import *
import os,sys,subprocess
cfg=load(D/'protocol.json')
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
for name in ['train.py','predict.py','replay.py','score.py']:
    child=subprocess.Popen([sys.executable,'-B',str(D/name)],env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
    put(D/(name[:-3]+'_process.json'),dict(pid=child.pid,parent_pid=os.getpid(),state='running'))
    rc=child.wait();put(D/(name[:-3]+'_process.json'),dict(pid=child.pid,parent_pid=os.getpid(),state='finished' if rc==0 else 'runtime_failed',returncode=rc));assert rc==0,(name,rc)
    if name=='train.py' and not load(D/'weight_seal.json')['fit_converged']:
        put(D/'result.json',dict(complete=True,utc=now(),decision='STOP_SINGLE_FIT_NOT_CONVERGED_NO_RETRY',offline_development_passed=False,
            fit=1,learned_score_rows=2038,source_calls=0,physics=0,navigation=0,learning_increment=False,old_budgets_reopened=False))
        break
