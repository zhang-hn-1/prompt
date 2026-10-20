"""Actual paired dynamics and source-consumer fork; independent of path_index."""
from pathlib import Path
import datetime,hashlib,json
import numpy as np
H=Path(__file__).resolve().parent
B=H/'observation_handoff_observe_run_recovery_v2';C=H/'qualified_hazard_navigation_run_v1'
status=json.loads((H/'qualified_hazard_navigation_status_v1.json').read_text());assert status['status']=='finished' and status['exit_code']==0
def loadjson(p):return json.loads(p.read_text())
b=np.load(B/'physics.npz');c=np.load(C/'physics.npz')
br=loadjson(B/'policy_records.json');cr=loadjson(C/'policy_records.json');n=min(len(br),len(cr))
first_action=next((i for i in range(n) if br[i]['local']['request']!=cr[i]['local']['request']),None)
first_map=None
for i in range(n):
    x=np.load(B/'maps'/f'frame_{i:03d}.npz');y=np.load(C/'maps'/f'frame_{i:03d}.npz')
    if not np.array_equal(x['labels'],y['labels']):first_map=i;break
boundary=min([i for i in [first_action,first_map] if i is not None],default=n)
fields=[k for k in b.files if k not in ['raw_simulation_time_s','raw_physics_step_index']]
assert set(fields)==set(c.files)-{'raw_simulation_time_s','raw_physics_step_index'}
last=5*boundary+1
physical={k:bool(np.array_equal(b[k][:last],c[k][:last])) for k in fields}
be=loadjson(B/'execution_states.json');ce=loadjson(C/'execution_states.json')
execution_prefix_equal=be[:boundary+1]==ce[:boundary+1]
policy_fields=['diagnostics','request','tracking','mapping']
policy_prefix_equal={k:all(br[i][k]==cr[i][k] for i in range(boundary)) for k in policy_fields}
raw_prefix_equal=all(br[i]['local']['request']==cr[i]['local']['request'] for i in range(boundary))
sourceframes=list(range(0,boundary+1,5))
source_exact=all((B/'sensor_packets'/f'source_{i:03d}.npz').read_bytes()==(C/'sensor_packets'/f'source_{i:03d}.npz').read_bytes() for i in sourceframes)
# NPZ compression metadata can differ; compare every original measurement field too.
source_fields_exact=True
for i in sourceframes:
    x=np.load(B/'sensor_packets'/f'source_{i:03d}.npz');y=np.load(C/'sensor_packets'/f'source_{i:03d}.npz')
    source_fields_exact &= set(x.files)==set(y.files) and all(np.array_equal(x[k],y[k]) for k in x.files)
common=min(len(b['simulation_time_s']),len(c['simulation_time_s']))
changed=np.any(b['applied_action'][:common]!=c['applied_action'][:common],axis=1)
task=loadjson(C/'task_episode.json')['actual_episode'];start=np.asarray(task['start_position_world_m'])[:2];goal=np.asarray(task['goal_position_world_m'])[:2];direction=(goal-start)/np.linalg.norm(goal-start)
def metrics(folder,ph):
    m=loadjson(folder/'metrics.json');xy=ph['root_position_world_m'][:,:2];progress=(xy-start)@direction;cross=(xy-start)@[-direction[1],direction[0]]
    contacts=loadjson(folder/'physics_contacts.json');contact=sum(bool(r['contact']) for r in contacts)
    hits=np.flatnonzero(xy[:,0]>=0.)
    return dict(termination=m['termination_reason'],elapsed_s=m['elapsed_s'],success=bool(m['success']),final_distance_m=float(np.linalg.norm(xy[-1]-goal)),actual_goal_line_progress_end_m=float(progress[-1]),actual_goal_line_progress_peak_m=float(progress.max()),path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),goal_line_cross_RMS_m=float(np.sqrt(np.mean(cross**2))),goal_line_cross_peak_m=float(np.abs(cross).max()),contact_samples=contact,actual_crossed_obstacle_X0=bool(len(hits)),first_X0_time_s=float(ph['simulation_time_s'][hits[0]]) if len(hits) else None)
bm,cm=metrics(B,b),metrics(C,c)
motion=cm['success'] and cm['final_distance_m']<=.2 or cm['actual_crossed_obstacle_X0'] and cm['actual_goal_line_progress_end_m']>bm['actual_goal_line_progress_end_m'] and cm['final_distance_m']<bm['final_distance_m']
navpass=bool(cm['contact_samples']==0 and cm['elapsed_s']<=35.00001 and motion)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),B0=bm,candidate=cm,first_actual_map_difference_frame=first_map,first_raw_request_difference_frame=first_action,prefix_boundary_frame=boundary,prefix_physics_samples=last,prefix_physical_fields_equal=physical,prefix_execution_states_equal=execution_prefix_equal,prefix_policy_fields_equal=policy_prefix_equal,prefix_requests_equal=raw_prefix_equal,prefix_original_source_bytes_equal=source_exact,prefix_original_source_fields_equal=bool(source_fields_exact),raw_clock_origin_difference_s=float(c['raw_simulation_time_s'][0]-b['raw_simulation_time_s'][0]),actual_applied_ticks_changed=int(changed.sum()),controlled_prefix_gate=all(physical.values()) and execution_prefix_equal and all(policy_prefix_equal.values()) and raw_prefix_equal and source_fields_exact,navigation_improvement_gate=navpass,decision='LOCAL_MOTION_GATE_PASS_PENDING_FULL_INTERFACE_SOURCE' if navpass else 'REJECT_NAVIGATION_IMPROVEMENT',not_formal_E0E1E2=True,knownpose_and_explicit_unknown_risk_priors=True,estimated_pose_not_admitted=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
f=H/'qualified_hazard_navigation_comparison_v1.json';assert not f.exists();f.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
