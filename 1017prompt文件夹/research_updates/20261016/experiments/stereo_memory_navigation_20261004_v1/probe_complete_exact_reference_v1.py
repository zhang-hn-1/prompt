"""Necessary current-input admission, never labels an alternate trajectory."""
from pathlib import Path
import sys,json,hashlib,datetime,numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from scripts.run_pointgoal_benchmark import load_action_protocol
from benchmark.policies import PolicyObservation
from scipy.spatial.transform import Rotation
from estimated_navigation_observation import EstimatedNavigationObservation
from outgoing_lookahead import make_endpoint_oracle
from exact_outgoing_lookahead import with_exact_outgoing_lookahead
from oracle_rgb_shadow import reference_from_arrays
from depth_local_curve import install_local_reference
D=H/'original_E1_repeated_exposure_memory_navigation_run_v1'
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text())
def plan(frame):
 d=json.loads((D/'local_plans'/f'frame_{frame:03d}.json').read_text());w=d['waypoints'];return reference_from_arrays([[v[k] for k in ['x_m','y_m','yaw_rad']] for v in w],[1 if v['drive_direction']=='forward' else -1 for v in w],[[v['main_command_rad_s'],v['auxiliary_angle_rad']] for v in w],d['path_length_m'],d['expanded_nodes'])
def tracker(candidate):
 p=load_action_protocol(P/'configs/data_contract.yaml');b=make_endpoint_oracle(p)
 if candidate:b=with_exact_outgoing_lookahead(type(b))(p,lookahead_m=.5)
 b.mapping_records=[];return b
B=tracker(False);C=tracker(True);trace=[];zero=np.zeros((480,640,3),np.uint8)
nav=EstimatedNavigationObservation();angular=np.load(D/'physics.npz')['angular_velocity_world_rad_s']
for i in range(375):
 if i in (60,155):
  install_local_reference(B,plan(i));install_local_reference(C,plan(i))
 h=np.load(D/'history'/f'frame_{i:03d}.npz');s=states[i];q=np.array(s['quaternion']);R=Rotation.from_quat(q[[1,2,3,0]]).as_matrix();xy=np.array(s['position'][:2]);original=PolicyObservation(zero,zero,h['proprio_history'],h['action_history'],xy,0.,h['quaternion_world_wxyz_history']);worldgoal=np.array([3.300000005960465,5.960465010446114e-9,.45]) if i<155 else np.r_[np.array([[w.x_m,w.y_m] for w in plan(155).waypoints])[-1],.45];obs,goal=nav.build(original,dict(time_s=s['time_s'],root_position_world_m=s['position'],root_rotation_world=R,velocity_world_m_s=s['velocity']),R.T@angular[i*5],worldgoal)
 if i<60:continue
 def act(t):
  a=t.act(obs,goal).copy();a[::2]=np.clip(a[::2],-.3,.5);return a
 old=act(B);new=act(C)
 if i<375:np.testing.assert_array_equal(old,rows[i]['request'])
 trace.append(dict(frame=i,B0_request=old.tolist(),candidate_request=new.tolist(),different=not np.array_equal(old,new),candidate_event=C.tracking_diagnostics.get('exact_outgoing_lookahead_event'),B0_state=B.diagnostics,candidate_state=C.diagnostics))
first=next((x['frame'] for x in trace if x['different']),None)
xy=np.array([[w.x_m,w.y_m] for w in plan(155).waypoints]);dd=np.diff(xy,axis=0);l2=(dd*dd).sum(1)
def error(pos):
 u=np.clip(((pos-xy[:-1])*dd).sum(1)/l2,0,1);return float(np.linalg.norm(xy[:-1]+u[:,None]*dd-pos,axis=1).min())
peak=max(error(np.array(states[i]['position'][:2])) for i in range(155,(first or 375)+1))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),domain='independent complete installed E1 reference2, cached known reference prior, not navigation',historical_B0_request_reconstruction_exact=True,first_request_fork_frame=first,pre_fork_cross_track_peak_m=peak,tracking_error_gate_m=.2,admission=bool(first is not None and peak<=.2),candidate_future_trajectory_not_available=True,old_R1_rejection_preserved=True,formal35_NOTRUN=True,whole_research_complete=False)
O=H/'complete_exact_reference_admission_v1';O.mkdir(exist_ok=False);(O/'trace.json').write_text(json.dumps(trace,indent=2)+'\n');(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
