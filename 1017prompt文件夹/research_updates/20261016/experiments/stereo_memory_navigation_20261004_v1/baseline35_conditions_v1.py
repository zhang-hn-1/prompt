"""TaskH knownpose baseline assessment; original source gates remain historical diagnostics."""
from pathlib import Path
import json,hashlib
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
def build_config(name):
 if name not in ('e0','e2'):raise ValueError('Only frozen new E0/E2 sequences; E1 uses already closed historical prefix')
 old=json.loads((H/'E1_continuous_tube_navigation_protocol_v1.json').read_text())
 keys=('source_sha256','input_sha256','scene_root','rig_extrinsics','static_masks','pid_config','inner_config')
 c={k:old[k] for k in keys};c['source_sha256']=c['source_sha256'].copy();c['input_sha256']=c['input_sha256'].copy()
 c.update(episode_manifest=str(H.parent/'visual_information_new_rig_20261004_v2/request_sequences'/f'{name}_manifest.json'),episode_id=f'golden_{name}_0001',occupancy=str(P/'data/benchmark_v1/occupancy'/('cluttered_hard_0_bodyband.npz' if name=='e0' else 'e2_corridor_bodyband.npz')),requests_max=350,sim_budget_s=35.,source_accuracy_is_diagnostic_only=True,knownpose_task_prior_baseline_not_strict_visual=True,no_controller_change=True,maximum_actual_sequences=1,no_retry=True,goal_threshold_m=.2,task='frozen binocular memory rollPID baseline35; knownpose flatground safe-start conditional task',front_view_existing_behavior_unchanged=True,old_source_and_navigation_rejections_preserved=True,success='measured XY goal <=.2 within35 and no actual contact',new_method_not_present=True)
 for f in [c['episode_manifest'],c['occupancy'],str(Path(__file__)),str(H/'run_baseline35_navigation_v1.py')]:c['input_sha256'][f]=hashlib.sha256(Path(f).read_bytes()).hexdigest()
 return c
