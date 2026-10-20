"""One offline bounded library/teacher budget; no restart of existing phase status."""
from pathlib import Path
import os,sys,json
from durable_stage_supervision_v1 import run_durable
H=Path(__file__).resolve().parent
if __name__=='__main__':
 phase=sys.argv[1]
 if phase not in ('predict','score'):raise ValueError(phase)
 review=json.loads((H/'episodic_execution_memory_prequery_review_v2.json').read_text())
 if any(review['severity_counts'][k] for k in ('Critical','Important')):raise ValueError('prequery review not admitted')
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
 result=run_durable([sys.executable,str(H/'run_episodic_execution_memory_v3.py'),phase],H/f'episodic_execution_{phase}_durable_v1.json',H/f'episodic_execution_{phase}_durable_v1.log',timeout=300,env=env)
 print(json.dumps(result));sys.exit(0 if result['status']=='finished' and result['child_returncode']==0 else 1)
