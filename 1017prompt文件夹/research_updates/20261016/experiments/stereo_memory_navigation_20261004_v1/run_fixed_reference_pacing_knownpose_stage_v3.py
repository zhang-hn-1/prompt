from pathlib import Path
import sys,os,json,hashlib,datetime
from durable_stage_supervision_v1 import run_durable
H=Path(__file__).resolve().parent
if __name__=='__main__':
 arm=sys.argv[1];assert arm in ('b0','paced')
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_pacing_ab')
 r=run_durable([sys.executable,str(H/'run_fixed_reference_pacing_knownpose_v3.py'),'--condition','e0','--arm',arm],H/('fixed_reference_pacing_knownpose_'+arm+'_status_v3.json'),H/('fixed_reference_pacing_knownpose_'+arm+'_v3.log'),timeout=1200,grace=45,env=env)
 D=H/('fixed_reference_pacing_knownpose_'+arm+'_run_v3')
 if D.exists():
  f=D/('artifact_manifest_post_close.json' if r['status']=='finished' else 'artifact_manifest_after_runtime_failure.json')
  f.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status=r,artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
 print(json.dumps(r),flush=True);sys.exit(0 if r['status']=='finished' else 1)
