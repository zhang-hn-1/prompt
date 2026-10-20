from pathlib import Path
from types import SimpleNamespace
import sys,json,numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1';sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'));sys.path.insert(0,'/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
from minimum_prefix_transition_consumer_v1 import measured_first_direction
from current_prefix_feasibility_v1 import current_two_forward_prefix
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());ph=np.load(D/'physics.npz');out=[];checks=0
for i,r in enumerate(rows[:507]):
 if not r.get('guard_measured_this_call') or not r.get('tube_viability_armed'):continue
 dec=r['reference_decision'];s=states[i];p=np.zeros((1,17));p[0,:3]=s['velocity'];p[0,12]=ph['joint_velocity_rad_s'][i*5,0];R=Rotation.from_quat(np.asarray(s['quaternion'])[[1,2,3,0]]).as_matrix();q=Rotation.from_matrix(R).as_quat()[[3,0,1,2]];yaw=float(np.arctan2(R[1,0],R[0,0]));ob=SimpleNamespace(proprio_history=p,yaw_world_rad=yaw,quaternion_world_wxyz_history=q[None]);elig=measured_first_direction(ob,dec['state_before']);checks+=1
 if r.get('first_primitive_direction_query') is not None:assert elig==r['first_primitive_direction_query']['direction'],i
 eligible=dec['decision']=='retain' and dec['tube_viability_armed_at_boundary'] and elig=='forward';v=None
 if eligible:
  m=np.load(D/'maps'/f'frame_{i:03d}.npz');g=RepeatedExposureMemoryGrid(m['origin'],10,static_scene_assumption=True);g.labels=m['labels'].copy();v=current_two_forward_prefix(g,s['position'][:2],yaw)
 out.append(dict(frame=i,measured_first_direction=elig,old_reference_decision=dec['decision'],eligibility_matches_actual_logged_query_when_available=True,consumer_evaluated=eligible,screen=v,changes_retention=eligible and not v['pass']))
result=dict(scope='HISTORICAL prefix<=506 developer replay only; same current state/old risk maps; no teacher/no policy act/no new physical data. One candidate and unchanged measured eligibility; no direction override to force intervention.',examined_boundaries=checks,evaluated=sum(r['consumer_evaluated'] for r in out),changes=[r['frame'] for r in out if r['changes_retention']],rows=out);(D/'current_prefix_consumer_preflight_v1.json').write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k!='rows'})
