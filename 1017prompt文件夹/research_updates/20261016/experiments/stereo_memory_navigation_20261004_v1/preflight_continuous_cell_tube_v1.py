"""Frozen history <=331; unique continuous screen, no re-act or future labels."""
from pathlib import Path
import sys,json,hashlib,numpy as np
H=Path(__file__).resolve().parent;sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
from continuous_cell_tube_v1 import continuous_cell_tube_screen
O=H/'continuous_cell_tube_preflight_v1';O.mkdir();out={};inputs={};sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name in ['minimum_prefix_transition_navigation_run_v1','E1_current_pixel_navigation_run_v1']:
 D=H/name;policy=D/'policy_records.json';rows=json.loads(policy.read_text());inputs[str(policy)]=sha(policy);comp=[]
 for i,row in enumerate(rows[:332]):
  if not row.get('guard_measured_this_call'):continue
  dec=row['reference_decision'];old=dec.get('tube_viability_screen')
  if old is None:continue
  mp=D/'maps'/f'frame_{i:03d}.npz';m=np.load(mp);inputs[str(mp)]=sha(mp);g=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);g.labels=m['labels'].copy();new=continuous_cell_tube_screen(g,dec['guard']);comp.append({'frame':i,'time_s':row['time_s'],'original_screen':old,'candidate_screen':new,'changes_screen':old['pass']!=new['pass'],'original_decision':dec['decision']})
 out[name]=comp
result={'scope':'HISTORICAL_PREFIX_NECESSARY_GEOMETRY_ONLY_NOT_CONTROL_AB','max_frame':331,'candidate_count':1,'no_future_teacher_or_policy_call':True,'old_margin_not_changed':True,'data_used_in_design_not_fresh':True,'datasets':out};(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');(O/'seal.json').write_text(json.dumps({'inputs':inputs,'outputs':{str(O/'assessment.json'):sha(O/'assessment.json')},'module_sha256':sha(H/'continuous_cell_tube_v1.py')},indent=2)+'\n');print({k:{'checks':len(v),'changes':[r['frame'] for r in v if r['changes_screen']]} for k,v in out.items()})
