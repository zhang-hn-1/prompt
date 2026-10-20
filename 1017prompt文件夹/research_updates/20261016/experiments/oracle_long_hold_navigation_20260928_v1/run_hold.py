import argparse,math,json
from pathlib import Path
import numpy as np
from helpers import write,snapshot,EventOracle
from benchmark.policies import EpisodeContext
from benchmark.drive_model import DriveModel
from training.action_protocol import load_action_protocol
from scripts.run_round4_fixed_reference import load_transformed_reference
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode

p=argparse.ArgumentParser();p.add_argument('--main',type=float,choices=[.5,1.5]);p.add_argument('--aux',type=float,choices=[-30,30]);a=p.parse_args()
HERE=Path(__file__).resolve().parent;out=HERE/f'hold_m{a.main}_a{a.aux}';out.mkdir(exist_ok=False)
ref,_=load_transformed_reference(HERE.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
protocol=load_action_protocol(DEFAULT_CONTRACT)
runtime=IsaacBenchmarkRuntime(IsaacRuntimeConfig(scene_root=DEFAULT_SCENE_ROOT.resolve(),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,record_video=False,occupancy_path=DEFAULT_OCCUPANCY.resolve(),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),diagnostic_open_plane=True,diagnostic_physics_trace=True,diagnostic_auxiliary_target_rate_rad_s=math.radians(30),diagnostic_auxiliary_velocity_limit_rad_s=.30))
policy=EventOracle(protocol,drive_model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE));policy.reset(EpisodeContext(episode_id=episode.episode_id,privileged=True,oracle_path_xy=ref.points_xy,action_protocol_id=protocol.protocol_id,kinodynamic_reference=ref.waypoints))
runtime.set_reference_visualization_path(ref.points_xy)
code=1
try:
 step=runtime.reset(episode)
 write(out/'config.json',{'main':a.main,'auxiliary_deg':a.aux,'baseline_s':5,'hold_s':30,'total_s':35,'control_scope':'prescribed commands only'})
 for block in range(350):
  target=np.array([a.main,0. if block<50 else math.radians(a.aux)],dtype=np.float32)
  step=runtime.execute(np.tile(target,(protocol.controls_per_block,1)).reshape(-1))
 np.savez(out/'physics.npz',**runtime.physics_diagnostics)
 write(out/'status.json',{'complete':True,'blocks':350,'physics_samples':len(runtime.physics_diagnostics['simulation_time_s'])})
 code=0
except BaseException:
 import traceback
 write(out/'failure.json',{'traceback':traceback.format_exc()})
 raise
finally:runtime.close(return_code=code)
