"""Two current-input geometry queries at sealed frame211; not another physical run."""
from pathlib import Path
import json,copy,types,dataclasses,hashlib,numpy as np
import execution_domain_four_point_consumer_v2 as consumer
import first_primitive_direction_planning_v1 as candidate
from guarded_reference import boundary_state
H=Path(__file__).resolve().parent
s=(H/'verify_observed_reference_tube_policy_causal_v2.py').read_text();s=s[:s.index('full=run(')]
needle=";action=acq.choose(t,state['position'][:2],float(np.linalg.norm(observation.proprio_history[-1,:2])),src)"
assert needle in s
s=s.replace(needle,"\n  if i==stop:return policy,observation,goal,src,packet\n  action=acq.choose(t,state['position'][:2],float(np.linalg.norm(observation.proprio_history[-1,:2])),src)")
ns={'__file__':str(H/'verify_observed_reference_tube_policy_causal_v2.py')};exec(compile(s,'<sealed causal prefix>','exec'),ns)
# Poison/truncate future feedback already enforced by the reused prefix reader.
p,o,g,src,packet=ns['run'](211,True)
state=boundary_state(p.tracker);main=float(o.proprio_history[-1,12]);speed=float(np.linalg.norm(o.proprio_history[-1,:2]))
assert state['active_direction']=='forward' and state['pending_direction'] is None and state['braking_state']=='tracking' and main>.1 and speed>.05
p.grid.set_source(src);p.grid.update_points(np.empty((0,3)),np.asarray(packet['root_xyz']),packet['time_s'])
xy=np.asarray(o.position_world_xy);from scipy.spatial.transform import Rotation
yaw=Rotation.from_quat(np.asarray(o.quaternion_world_wxyz_history[-1])[[1,2,3,0]]).as_euler('xyz')[2]
# Preserve the exact consumer's map/route/crop/target/clearance, changing only planner domain.
globals_copy=dict(consumer.local_curve_for_direction_tube.__globals__)
def restricted(req,drive,**kwargs):
 req=candidate.KinodynamicPlanRequest(**vars(req))
 return candidate.plan_kinodynamic_path(req,drive,first_primitive_direction='forward',**kwargs)
globals_copy['plan_kinodynamic_path']=restricted
restricted_consumer=types.FunctionType(consumer.local_curve_for_direction_tube.__code__,globals_copy)
before=p.grid.labels.copy();out=[]
for name,fn in [('B0',consumer.local_curve_for_direction_tube),('first_forward',restricted_consumer)]:
 plan,meta=fn(p.grid,xy,yaw,g,p.tracker.drive_model,initial_direction='forward')
 out.append(dict(arm=name,meta=meta,plan=None if plan is None else dataclasses.asdict(plan)))
 np.testing.assert_array_equal(before,p.grid.labels)
assert out[0]['meta']['local_target']==out[1]['meta']['local_target']
assert out[0]['meta']['planning_clearance_m']==out[1]['meta']['planning_clearance_m']
base=out[0]['plan'];trial=out[1]['plan']
admit=trial is not None and trial['waypoints'][0]['main_command_rad_s']>0
r=dict(decision='PASS_NECESSARY_FIRST_PRIMITIVE_GEOMETRY_ONLY' if admit else 'REJECT_FIRST_PRIMITIVE_LOCAL_DOMAIN',frame=211,actual_measured_main_rad_s=main,actual_XY_speed_m_s=speed,boundary_before=state,queries=out,map_unchanged=True,same_target_and_margin=True,current_and_past_only=True,not_dynamic_prediction=True,not_another_online_run=True,not_navigation_admission=True,original35_NOTRUN=True,whole_research_complete=False)
(H/'first_primitive_direction_assessment_v1.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({k:v for k,v in r.items() if k!='queries'},indent=2));print([(a['arm'],a['meta']['status'],a['meta'].get('expanded_nodes'),None if a['plan'] is None else [w['drive_direction'] for w in a['plan']['waypoints']]) for a in out])
