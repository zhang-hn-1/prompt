"""Dependency recovery only: pre-fit sklearn import failure did not train a model."""
from pathlib import Path
import os,sys,json
from durable_stage_supervision_v1 import run_durable
from learned_memory_risk_identity_v1 import verify_prefit_identity,sha
H=Path(__file__).resolve().parent
if __name__=='__main__':
 stage=sys.argv[1];assert stage in ['fit','predict','assess'];verify_prefit_identity(H)
 protocol=json.loads((H/'learned_memory_risk_dependency_recovery_protocol_v2.json').read_text())
 for path,digest in protocol['input_sha256'].items():assert sha(path)==digest,path
 script={'fit':'fit_learned_memory_risk_v1.py','predict':'predict_learned_memory_risk_holdout_v1.py','assess':'assess_learned_memory_risk_holdout_v1.py'}[stage]
 if stage=='fit':
  previous=json.loads((H/'learned_memory_risk_fit_status_v2.json').read_text());assert previous['status']=='runtime_failed' and previous['child_returncode']==1
  assert "KeyError: 'status'" in (H/'learned_memory_risk_fit_v2.log').read_text()
  old=json.loads((H/'learned_memory_risk_fit_status_v1.json').read_text());assert old['status']=='runtime_failed' and old['child_returncode']==1
  assert 'ModuleNotFoundError' in (H/'learned_memory_risk_fit_v1.log').read_text() and not (H/'learned_memory_risk_model_v1.json').exists()
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(H)+':'+str(H/'learning_risk_dependencies_v1')+':/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
 result=run_durable([sys.executable,str(H/script)],H/f'learned_memory_risk_{stage}_status_v3.json',H/f'learned_memory_risk_{stage}_v3.log',timeout=900,grace=15,env=env)
 print(json.dumps(result));sys.exit(0 if result['status']=='finished' and result['child_returncode']==0 else 2)
