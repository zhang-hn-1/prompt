from pathlib import Path
import json,hashlib,datetime,sys,inspect,ast
import numpy as np
H=Path(__file__).resolve().parent;W=H.parent.parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P));sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
from oracle_rgb_shadow import make_oracle,reference_from_arrays
from training.action_protocol import load_action_protocol
from scripts.camera_rig import camera_rig_metadata
old=W/'experiments/oracle_feasible_route_20260929_v9/navigation_hybrid_golden_golden_e1_0001/episode_golden_e1_0001';hybrid=old.parent.parent/'hybrid';s=np.load(old/'states.npz');decision=json.loads((old.parent/'feedback/reference_decision.json').read_text());metrics=json.loads((old/'metrics.json').read_text());proto=load_action_protocol(P/'configs/data_contract.yaml');policy=make_oracle(proto)
np.savez_compressed(H/'oracle_rgb_shadow_reference.npz',pose=s['kinodynamic_reference_pose'],direction=s['kinodynamic_reference_drive_direction'],command=s['kinodynamic_reference_command'])
plan=reference_from_arrays(s['kinodynamic_reference_pose'],s['kinodynamic_reference_drive_direction'],s['kinodynamic_reference_command'],decision['frozen_length_m'],metrics['certification']['expanded_nodes']);assert len(plan.waypoints)==30 and plan.direction_changes==0
catalog=[];environments=0
for f in (W/'experiments').glob('*/**/environment.json'):
 environments+=1
 try:e=json.loads(f.read_text())
 except (OSError,ValueError):continue
 if e.get('policy')=='oracle' and e.get('camera_rig'):catalog.append(str(f))
assert not catalog
prior=json.loads((H/'point_support_verification.json').read_text())
for f,h in prior['source_and_artifact_sha256'].items():assert sha(f)==h,f
sync=json.loads((H/'point_support_document_sync.json').read_text())
for f,h in sync['document_sha256'].items():assert sha(f)==h,f
source={f:sha(f) for f in json.loads((H/'active_observation_protocol.json').read_text())['source_sha256'] if not f.endswith('.md')}
source.update({str(f):sha(f) for f in [H/'oracle_rgb_shadow.py',H/'test_oracle_rgb_shadow.py',H/'run_oracle_rgb_shadow.py',H/'run_oracle_rgb_shadow_stage.py',H/'seal_oracle_rgb_shadow.py',H/'oracle_rgb_shadow_reference.npz',H/'point_support_shadow.py',H/'feature_tracks.py',H/'active_observation.py',hybrid/'gated_speed.py',hybrid/'bank_mapping.py',hybrid/'bank_mapping.json',hybrid/'roll_pid.py',hybrid/'selected_candidate.json',old/'states.npz',old/'metrics.json',old/'manifest.json',old/'environment.json',old.parent/'feedback/reference_decision.json',W/'docs/superpowers/plans/2026-10-05-oracle-online-rgb-shadow.md',P/'benchmark/runner.py',P/'benchmark/policies.py',P/'benchmark/isaac_runtime.py',P/'configs/data_contract.yaml',P/'configs/benchmark_v1/drive_dynamics_v1.json']})
for f,comparison in json.loads((H/'reusable_oracle_audit.json').read_text())['historical_core_source_comparison'].items():
 if '/hybrid/' in f:assert sha(f)==comparison['recorded_sha256'],f
mro=[dict(class_name=c.__qualname__,module=c.__module__,source_file=inspect.getsourcefile(c),source_sha256=sha(inspect.getsourcefile(c))) for c in type(policy).__mro__ if c is not object]
report=dict(checked_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),environment_catalog_count=environments,currentrig_oracle_catalog=catalog,prior_artifacts_checked=len(prior['source_and_artifact_sha256']),prior_documents_checked=5,policy_mro=mro,act=dict(module=policy.act.__func__.__module__,qualname=policy.act.__func__.__qualname__,source_file=inspect.getsourcefile(policy.act)),historical_identity_limit='old runtime and pointgoal source differed; old environment lacks rig. Cached archived selected reference only, no hybrid search repeated.',reference_waypoints=30,reference_length_m=8.7,reference_direction_changes=0,reference_source='exact archived hybrid chosen frozen_measured_time_with_reserve, not actual future trajectory',camera_rig=camera_rig_metadata(),source_sha256=source)
(H/'oracle_rgb_shadow_identity_audit.json').write_text(json.dumps(report,indent=2)+'\n')
source[str(H/'oracle_rgb_shadow_identity_audit.json')]=sha(H/'oracle_rgb_shadow_identity_audit.json')
p=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),manifest=str(old/'manifest.json'),episode_id='golden_e1_0001',maximum_sequences=1,episode_budget_s=35.,max_action_blocks=350,goal_tolerance_m=.2,collision_mode='physical_contact_v2',reference_length_m=8.7,reference_expanded_nodes=483,auxiliary_velocity_limit_rad_s=.3,lookahead_m=.5,pid_gains=json.loads((hybrid/'selected_candidate.json').read_text())['gains'],camera_rig=camera_rig_metadata(),lighting='fixed_camera_fill_v1',shadow_scope='readonly RGB+known-pose diagnostic; no truth/map/reference/control inputs, same rejected ORB/10s provisional endpoint-map',shadow_admitted=False,source_gate_passed=False,consumer_gate_passed=False,formal_matrix=False,new_control_candidate=False,fresh_environment_generalization=False,source_sha256=source)
(H/'oracle_rgb_shadow_protocol.json').write_text(json.dumps(p,indent=2)+'\n');print(json.dumps({k:v for k,v in p.items() if k!='source_sha256'},indent=2));print('SEALED_SOURCES',len(source),'CLASS',mro)
