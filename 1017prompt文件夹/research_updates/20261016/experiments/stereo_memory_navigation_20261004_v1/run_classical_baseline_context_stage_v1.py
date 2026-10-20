"""One whole35 online read-only context/reproduction validation, no candidate rerun."""
from pathlib import Path
import sys,os,json,hashlib,datetime
from durable_stage_supervision_v1 import run_durable
H=Path(__file__).resolve().parent
if __name__=='__main__':
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm',MPLCONFIGDIR='/tmp/mpl_baseline_context')
 D=H/'classical_baseline_context_e0_run_v1'
 r=run_durable([sys.executable,str(H/'run_classical_baseline_repro_v1.py'),'--condition','e0','--output-run',str(D),'--request-context','on'],H/'classical_baseline_context_e0_status_v1.json',H/'classical_baseline_context_e0_v1.log',timeout=1200,grace=45,env=env)
 if D.exists():
  f=D/('artifact_manifest_post_close.json' if r['status']=='finished' else 'artifact_manifest_after_runtime_failure.json')
  f.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status=r,artifact_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in D.rglob('*') if p.is_file()}),indent=2)+'\n')
 print(json.dumps(r),flush=True);sys.exit(0 if r['status']=='finished' else 1)
