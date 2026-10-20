from pathlib import Path
import json,sys,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');H=W/'experiments/oracle_feasible_route_20260929_v9/hybrid';sys.path[:0]=[str(P),str(H)]
from bank_mapping import BankMap
from scripts.run_pointgoal_benchmark import load_action_protocol
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT
model=BankMap(json.loads((H/'bank_mapping.json').read_text()));p=load_action_protocol(DEFAULT_CONTRACT.resolve());R=D/'candidate_e1_run';rows=json.loads((R/'policy_records.json').read_text());requests=np.load(R/'requests.npz')['requests'];count=0;changed=0;below=0;noise=0
for i,row in enumerate(rows):
 a=row.get('current_speed_yaw_target')
 if not a:continue
 bank=model.query(a['main_request_rad_s'],a['measured_speed_m_s'],a['curvature_1_m'])['encoded_aux_rad'];new=np.float32(np.clip(bank+a['new_correction_rad'],p.minimum[1],p.maximum[1]));old=np.float32(np.clip(bank+a['old_correction_rad'],p.minimum[1],p.maximum[1]));assert np.all(requests[i,1::2]==new),(i,requests[i,1::2],new);count+=1
 if new!=old:
  changed+=1
  if a['measured_speed_m_s']<.2:below+=1
  else:noise+=1
r=dict(issued_request_blocks_verified=count,same_current_state_counterfactual_aux_changes=changed,below020_changes=below,above020_precision_only_changes=noise,scope='actual candidate requests verified to bank map+new correction; legacy counterfactual uses same candidate state, not actual baseline trajectory; physical transport independently audited')
(D/'request_effect.json').write_text(json.dumps(r,indent=2)+'\n');print(r)
