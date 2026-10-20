"""Author hash/count/algebra verification; no planner search or physics."""
from pathlib import Path
import json,hashlib,datetime,sys,math
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P));sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'time_cost_protocol.json').read_text());a=json.loads((H/'time_cost_assessment.json').read_text())
for group in ['source_sha256','input_sha256']:
 for f,d in p[group].items():assert sha(f)==d,f
assert p['sealed_utc']<a['started_utc']<a['search_started_utc']<a['search_completed_utc']<a['completed_utc']
assert a['target_candidate_hybrid_searches']==1 and a['target_original_hybrid_searches']==0 and a['shared_grid_route_constructions']==1
assert a['arms'][1]['status']=='node_budget_exhausted' and a['arms'][1]['expanded_nodes']==3000
assert not a['cpu_geometry_eligibility_pass'] and not a['physical_model_admitted'] and not a['navigation_admitted'] and not a['whole_research_complete'] and a['automation']=='ACTIVE'
assert all(a[k]==0 for k in ['new_physical_sequences','policy_act_calls','model_fits'])
old=json.loads((H/'nominal_support_assessment.json').read_text())['arms'][0];b=a['arms'][0]
for k in ['pose','command','direction','expanded_nodes','path_length_m','direction_changes','objective','primitive_checks','local_goal_distance_m']:assert b[k]==old[k],k
pr=b['time_proxies'];weighted=0.;plain=0.;chord=0.
for r in pr['rows']:
 speed=r['nominal_v1_speed_m_s'];assert speed>.05
 weighted+=r['arc_length_m']*pr['maximum_v1_speed_m_s']/speed*(1.25 if r['reverse'] else 1.)+.5*r['entering_direction_change'];plain+=r['arc_length_m']/speed;chord+=r['chord_length_m']/speed
assert abs(weighted/pr['maximum_v1_speed_m_s']-pr['weighted_arc_time_proxy_s'])<1e-12
assert abs(plain-pr['unweighted_arc_time_proxy_s'])<1e-12
assert abs(chord+2-pr['archived_hybrid_chord_plus_internal_switch_proxy_s'])<1e-12
# Mechanical expected diff: no replacement geometry, gains, root latch or speed strategy.
s=(H/'direction_root_planning.py').read_text();n=s.replace('from benchmark.drive_model import DriveModel','from benchmark.drive_model import DriveModel\nfrom nominal_time_cost import nominal_edge_cost\nfrom direction_root_planning import KinodynamicPlanRequest, KinodynamicWaypoint, KinodynamicPlanResult');i=n.index('@dataclass(frozen=True)');j=n.index('StateKey =',i);n=n[:i]+n[j:];n=n.replace('    primitive_templates: list[','    maximum_speed = max(sample.speed_m_s for sample in drive_model.steering_samples)\n\n    primitive_templates: list[');n=n.replace('            edge_cost = request.primitive_length_m','            edge_cost = nominal_edge_cost(\n                request.primitive_length_m, drive_model.speed(main, auxiliary), maximum_speed\n            )');assert n==(H/'time_cost_planning.py').read_text()
import direction_root_planning as root,time_cost_planning as tc
assert root.KinodynamicPlanRequest is tc.KinodynamicPlanRequest and root.KinodynamicWaypoint is tc.KinodynamicWaypoint and root.KinodynamicPlanResult is tc.KinodynamicPlanResult
from benchmark.drive_model import load_drive_model
# Only template geometry identity introspection, no search.
from outgoing_lookahead import make_endpoint_oracle
from training.action_protocol import load_action_protocol
model=make_endpoint_oracle(load_action_protocol(P/'configs/data_contract.yaml')).drive_model;templates=[]
for sign in [1,-1]:
 for main in root.MAIN_MAGNITUDES_RAD_S:
  for aux in root.AUXILIARY_ANGLES_RAD:
   kwargs=dict(start_pose=(0,0,0),signed_arc_length_m=sign*.3,curvature_1_m=model.curvature(sign*main,aux),sample_spacing_m=.05)
   np.testing.assert_array_equal(root.sample_motion_primitive(**kwargs),tc.sample_motion_primitive(**kwargs));templates.append(dict(main=sign*main,aux=aux,curvature=model.curvature(sign*main,aux),speed=model.speed(sign*main,aux)))
assert len(templates)==42 and min(t['speed'] for t in templates)>.05
for path,phrase in [('time_cost_red.log','9 failed, 1 passed'),('time_cost_green.log','10 passed'),('time_cost_suite.log','237 passed')]:assert phrase in (H/path).read_text()
r=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),input_checks=len(p['input_sha256']),mechanical_diff_exact=True,original_request_type_aliases_exact=True,template_geometry_checks=templates,archived_B0_exact=True,time_proxy_algebra_exact=True,chronology_pass=True,assessment_sha256=sha(H/'time_cost_assessment.json'),verification_searches=0,new_physical_sequences=0,unique_candidate_target_searches=1,candidate_cpu_eligibility='REJECT',whole_research_complete=False,automation='ACTIVE')
(H/'time_cost_verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='template_geometry_checks'},indent=2))
