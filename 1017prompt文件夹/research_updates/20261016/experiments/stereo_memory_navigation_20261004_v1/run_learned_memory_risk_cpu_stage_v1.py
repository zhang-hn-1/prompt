from pathlib import Path
import os,sys,json
from durable_stage_supervision_v1 import run_durable
from learned_memory_risk_identity_v1 import verify_prefit_identity,sha
H=Path(__file__).resolve().parent
if __name__=='__main__':
 stage=sys.argv[1];assert stage in ['fit','predict','assess'];verify_prefit_identity(H)
 script={'fit':'fit_learned_memory_risk_v1.py','predict':'predict_learned_memory_risk_holdout_v1.py','assess':'assess_learned_memory_risk_holdout_v1.py'}[stage]
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(H)+':/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
 result=run_durable([sys.executable,str(H/script)],H/f'learned_memory_risk_{stage}_status_v1.json',H/f'learned_memory_risk_{stage}_v1.log',timeout=900,grace=15,env=env)
 print(json.dumps(result));sys.exit(0 if result['status']=='finished' and result['child_returncode']==0 else 2)
