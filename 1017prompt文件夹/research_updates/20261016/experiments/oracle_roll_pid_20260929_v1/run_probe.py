import argparse,math,json
from pathlib import Path
import numpy as np
from roll_pid import install_runtime
from training.action_protocol import load_action_protocol
from scripts.run_round4_fixed_reference import load_transformed_reference
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
p=argparse.ArgumentParser();p.add_argument('--main',type=float,required=True);p.add_argument('--mode',choices=['b0','pid'],required=True);p.add_argument('--gains',default='.8,.1,.35');p.add_argument('--tag',default='initial');a=p.parse_args()
h=Path(__file__).resolve().parent;out=h/f'probe_{a.tag}_{a.mode}_{a.main}';out.mkdir(exist_ok=False)
ref,_=load_transformed_reference(h.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
protocol=load_action_protocol(DEFAULT_CONTRACT)
cls=IsaacBenchmarkRuntime if a.mode=='b0' else install_runtime(IsaacBenchmarkRuntime,*map(float,a.gains.split(',')),output=out)
runtime=cls(IsaacRuntimeConfig(scene_root=DEFAULT_SCENE_ROOT.resolve(),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,record_video=False,occupancy_path=DEFAULT_OCCUPANCY.resolve(),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),diagnostic_open_plane=True,diagnostic_physics_trace=True,diagnostic_auxiliary_target_rate_rad_s=math.radians(30),diagnostic_auxiliary_velocity_limit_rad_s=.30))
runtime.set_reference_visualization_path(ref.points_xy)
code=1
try:
 runtime.reset(episode)
 for block in range(350):
  deg=0 if block<50 or block>=250 else (20 if block<150 else -20)
  target=np.array([a.main,math.radians(deg)],dtype=np.float32)
  runtime.execute(np.tile(target,(protocol.controls_per_block,1)).reshape(-1))
 np.savez(out/'physics.npz',**runtime.physics_diagnostics)
 (out/'config.json').write_text(json.dumps(vars(a),indent=2));code=0
finally:runtime.close(return_code=code)
