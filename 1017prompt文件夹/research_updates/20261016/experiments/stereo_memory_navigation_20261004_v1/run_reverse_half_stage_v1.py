from pathlib import Path
import sys,os,json,hashlib,datetime
from durable_stage_supervision_v1 import run_durable
H=Path(__file__).resolve().parent
if __name__=='__main__':
    arm=sys.argv[1];assert arm in ('development','validation')
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_reverse_half')
    r=run_durable([sys.executable,str(H/'run_reverse_half_v1.py'),'--arm',arm],H/('reverse_half_'+arm+'_status_v1.json'),H/('reverse_half_'+arm+'_run_v1.log'),timeout=900,grace=45,env=env)
    D=H/('reverse_half_'+arm+'_run_v1')
    if D.exists():
        f=D/('post_close_manifest.json' if r['status']=='finished' else 'failure_manifest.json')
        f.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status=r,artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
    print(json.dumps(r),flush=True);sys.exit(0 if r['status']=='finished' else 1)
