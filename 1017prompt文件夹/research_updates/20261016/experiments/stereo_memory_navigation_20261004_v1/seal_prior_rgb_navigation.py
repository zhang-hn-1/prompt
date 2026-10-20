from pathlib import Path
import json,hashlib,datetime,ast,sys,dataclasses
H=Path(__file__).resolve().parent;W=H.parent.parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P));sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();old=json.loads((H/'depth_local_curve_protocol.json').read_text());source={f:sha(f) for f in old['source_sha256']}
for f in H.glob('*.py'):source[str(f)]=sha(f)
for f in P.rglob('*.py'):
 if any(x in f.parts for x in ['.git','__pycache__']):continue
 source[str(f)]=sha(f)
for f in [W/'docs/superpowers/specs/2026-10-07-prior-rgb-local-navigation-design.md',W/'docs/superpowers/plans/2026-10-07-prior-rgb-local-navigation.md',H/'third_party/fast_foundation_stereo/weights/23-36-37/model_best_bp2_serialize.pth',H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz']:
 assert f.is_file(),f;source[str(f)]=sha(f)
for f in (H/'third_party/fast_foundation_stereo').rglob('*.py'):source[str(f)]=sha(f)
from benchmark.manifest import load_manifest
from prior_rgb_navigation import short_goal
original=next(e for e in load_manifest(old['manifest']).episodes if e.episode_id=='golden_e1_0001');local=dataclasses.replace(original,episode_id='prior_rgb_e1_local2m_current',goal_position_world_m=tuple(short_goal(original.start_position_world_m,original.goal_position_world_m)),euclidean_distance_m=min(2.,original.euclidean_distance_m),oracle_path_length_m=2.)
for n in ['prior_rgb_navigation.py','run_prior_rgb_navigation.py','run_prior_rgb_navigation_stage.py']:ast.parse((H/n).read_text())
source[old['manifest']]=sha(old['manifest']);p=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),manifest=old['manifest'],episode_id='golden_e1_0001',arms=['current','history'],maximum_sequences=2,original_budget_s=35.,local_goal_m=2.,goal_tolerance_m=.2,RGB_source='frozenFast B0,eager8iters416maxdisp1.5LR',infer_every_capture_frames=5,maximum_inferences_per_arm=71,knownpose_privileged=True,global_reference_or_map_online=False,flat_ground_task_assumption=True,initial_safe_zone_m=.7,task_priors_observed_free=False,unknown_request_main_cap_rad_s=.3,observed_hazard_clearance_m=.6,history_TTL_s=10.,old_source_admitted=False,formal_navigation=False,whole_research_complete=False,original_episode=original.to_dict(),local_episode_example=local.to_dict(),source_sha256=source)
f=H/'prior_rgb_navigation_protocol.json';assert not f.exists();f.write_text(json.dumps(p,indent=2)+'\n');print('presealed',len(source),'local goal',local.goal_position_world_m,'original start',local.start_position_world_m)
