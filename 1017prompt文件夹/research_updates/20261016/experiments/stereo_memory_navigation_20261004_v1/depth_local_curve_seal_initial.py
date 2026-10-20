from pathlib import Path
import sys,json,hashlib,datetime,inspect,ast
H=Path(__file__).resolve().parent;W=H.parent.parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
from depth_local_curve import DepthLocalCurvePolicy
from training.action_protocol import load_action_protocol
from scripts.camera_rig import camera_rig_metadata
prior=json.loads((H/'oracle_rgb_shadow_final_identity.json').read_text())
for f,s in prior['final_artifact_sha256'].items():assert sha(f)==s,f
sync=json.loads((H/'oracle_rgb_shadow_document_sync.json').read_text())
for f,s in sync['document_sha256'].items():
 if sha(f)==s:continue
 assert Path(f)==H/'progress.md',f
 before=Path(f).read_bytes().split(b'\n\n03:39UTC Ruling:')[0]
 assert hashlib.sha256(before).hexdigest()==s
 (H/'depth_local_curve_pre_stage_ledger.md').write_bytes(before)
old=json.loads((H/'oracle_rgb_shadow_protocol.json').read_text())
source={f:sha(f) for f in old['source_sha256']}
for f in [H/'depth_local_curve.py',H/'test_depth_local_curve.py',H/'run_depth_local_curve.py',H/'run_depth_local_curve_stage.py',H/'seal_depth_local_curve.py',H/'provenance_grid.py',H/'spatial_memory.py',H/'risk_grid.py',W/'experiments/visible_surface_connectivity_20261004_v1/local_map.py',P/'benchmark/kinodynamic_planning.py',P/'benchmark/grid_planning.py',W/'docs/superpowers/specs/2026-10-05-depth-local-curve-design.md',W/'docs/superpowers/plans/2026-10-05-depth-local-curve.md']:
 source[str(f)]=sha(f)
policy=DepthLocalCurvePolicy(load_action_protocol(P/'configs/data_contract.yaml'))
audit=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),outer_class=str(type(policy)),outer_runner_global_privilege=False,sensor_layer_privileged=True,inner_mro=[dict(name=c.__qualname__,module=c.__module__,file=inspect.getsourcefile(c),sha256=sha(inspect.getsourcefile(c))) for c in type(policy.tracker).__mro__ if c is not object],inner_act=str(policy.tracker.act.__func__),old_information_main_cap_retained=False,nominal_geometric_planner='existing plan_kinodynamic_path, not learned arbitrary-state predictor',prior_artifact_checks=len(prior['final_artifact_sha256']),prior_documents_checks=5)
(H/'depth_local_curve_identity_audit.json').write_text(json.dumps(audit,indent=2)+'\n');source[str(H/'depth_local_curve_identity_audit.json')]=sha(H/'depth_local_curve_identity_audit.json')
for f in [H/'depth_local_curve.py',H/'run_depth_local_curve.py',H/'run_depth_local_curve_stage.py']:ast.parse(f.read_text())
p=dict(sealed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),manifest=old['manifest'],episode_id='golden_e1_0001',maximum_sequences=1,episode_budget_s=35.,max_action_blocks=350,goal_tolerance_m=.2,collision_mode='physical_contact_v2',settling_prefix_s=6.,plan_period_s=.5,local_target_arc_m=2.4,local_crop_half_m=4.,node_budget_per_plan=3000,nominal_primitives_m=.3,observed_hazard_clearance_m=.6,pid_gains=old['pid_gains'],auxiliary_velocity_limit_rad_s=.3,camera_rig=camera_rig_metadata(),lighting=old['lighting'],task='explicit unknown-risk privileged depth+knownpose local-curve package',old_information_policy_main_cap_retained=False,successful_oracle_scheduler_unchanged=True,global_reference_or_map_online=False,full_3d_certified=False,rgb_admitted=False,formal_matrix=False,whole_research_complete=False,acceptance='original35s actual goalentry <=.2m, no physicalcontact, all causal/interface/evidence tests pass; only narrow privileged layer',source_sha256=source)
(H/'depth_local_curve_protocol.json').write_text(json.dumps(p,indent=2)+'\n');print(json.dumps({k:v for k,v in p.items() if k!='source_sha256'},indent=2));print('SOURCE_COUNT',len(source))
