from replay import *
import subprocess,sys,os
cfg=load(D/'protocol.json')
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
for name in ['replay.py','score.py']:
    child=subprocess.Popen([sys.executable,'-B',str(D/name)],env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'))
    put(D/(name.split('.')[0]+'_process.json'),dict(pid=child.pid,parent_pid=os.getpid(),script=name,state='running'))
    rc=child.wait();put(D/(name.split('.')[0]+'_process.json'),dict(pid=child.pid,parent_pid=os.getpid(),script=name,state='finished' if rc==0 else 'runtime_failed',returncode=rc));assert rc==0,(name,rc)
