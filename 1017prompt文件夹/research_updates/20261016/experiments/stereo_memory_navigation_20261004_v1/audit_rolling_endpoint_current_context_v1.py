"""Per-row current/past reconstruction only; no act, future truth or new motion label."""
from pathlib import Path
from types import SimpleNamespace as N
import json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
from rolling_endpoint_qualification_v1 import rolling_context
H=Path(__file__).parent;D=H/'measured_speed_navigation_e0_paced_run_v1';rows=json.loads((D/'policy_records.json').read_text());ss=json.loads((D/'states.json').read_text());goal=np.array(json.loads((D/'task_episode.json').read_text())['actual_episode']['goal_position_world_m'][:2]);plans={int(p.stem.split('_')[-1]):json.loads(p.read_text()) for p in (D/'local_plans').glob('frame_*.json')};current=None;contexts=[]
for r in rows:
 f=r['frame']
 if f in plans:current=plans[f]
 if not r.get('planner_act_called'):continue
 if r['measured_speed_control_reasons']!=['preserve_original_within_same_reserve']:continue
 assert current is not None and current['frame']<=f
 R=Rotation.from_quat(np.array(ss[f]['quaternion'])[[1,2,3,0]]).as_matrix();yaw=float(np.arctan2(R[1,0],R[0,0]));xy=np.array(ss[f]['position'][:2]);c,s=np.cos(yaw),np.sin(yaw);g=np.array([[c,s],[-s,c]])@(goal-xy)
 plan=N(status=current['status'],direction_changes=current['direction_changes'],waypoints=[N(**w) for w in current['waypoints']]);p=N(time=r['time_s'],plan=plan,plan_meta=r['plan'],reference_decision=r['reference_decision'],tracker=N(_reference_xy=np.array(r['installed_reference_xy_after_act'])))
 context=rolling_context(p,N(position_world_xy=xy,yaw_world_rad=yaw),g);contexts.append(dict(frame=f,reference_version=r['local_reference_version'],current_plan_frame=current['frame'],context=context))
by={}
for v in sorted(set(r['reference_version'] for r in contexts)):
 group=[r for r in contexts if r['reference_version']==v];by[str(v)]=dict(rows=len(group),current_nominal_continuation_eligible=sum(r['context']['eligible'] for r in group),reasons=sorted(set(z for r in group for z in r['context']['reasons'])))
result=dict(rows=contexts,by_reference=by,additional_physics=0,actual_act_calls=0,no_future_motion_labels=True,not_online_validation=True,not_prediction_or_navigation_admission=True,input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'policy_records.json',D/'states.json',D/'task_episode.json',*sorted((D/'local_plans').glob('*.json'))]},consumer_sha256=hashlib.sha256((H/'rolling_endpoint_qualification_v1.py').read_bytes()).hexdigest());out=H/'rolling_endpoint_current_context_audit_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(by))
