import argparse,math,json
from pathlib import Path
import numpy as np
from helpers import write,snapshot,EventOracle
from benchmark.policies import EpisodeContext
from benchmark.drive_model import DriveModel
from training.action_protocol import load_action_protocol
from scripts.run_round4_fixed_reference import load_transformed_reference
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
from geometry import relocate
p=argparse.ArgumentParser();p.add_argument('--kind',choices=['LR','RL']);a=p.parse_args()
HERE=Path(__file__).resolve().parent;out=HERE/(a.kind+'_validation');out.mkdir(exist_ok=False)
ref,_=load_transformed_reference(HERE.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
protocol=load_action_protocol(DEFAULT_CONTRACT)
runtime=IsaacBenchmarkRuntime(IsaacRuntimeConfig(scene_root=DEFAULT_SCENE_ROOT.resolve(),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,record_video=False,occupancy_path=DEFAULT_OCCUPANCY.resolve(),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),diagnostic_open_plane=True,diagnostic_physics_trace=True,diagnostic_auxiliary_target_rate_rad_s=math.radians(30),diagnostic_auxiliary_velocity_limit_rad_s=.30))
policy=EventOracle(protocol,drive_model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE));policy.reset(EpisodeContext(episode_id=episode.episode_id,privileged=True,oracle_path_xy=ref.points_xy,action_protocol_id=protocol.protocol_id,kinodynamic_reference=ref.waypoints))
angles=np.radians([18.425763799461098,-24.4231088098262]);angles=angles if a.kind=='LR' else angles[::-1]
runtime.set_reference_visualization_path(ref.points_xy)
code=1
try:
 step=runtime.reset(episode)
 write(out/'config.json',{'kind':a.kind,'main':1.5,'angles':angles,'boundary_s':8.,'transition_s':12.,'control_scope':'prescribed open-loop commands; EventOracle never act-called'})
 for block in range(200):
  if block in [80,120,160]:
   state=snapshot(runtime,policy,step);state.pop('policy');write(out/f'boundary_{block}.json',state)
   if block==160:
    opposite=a.kind
    source=HERE.parent/'oracle_measured_turn_primitives_20260928_v1'/f'{opposite}_measure'
    z=np.load(source/'physics.npz');q=state['quaternion'];yaw=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))
    prediction=relocate(z['position_world_xy'][400:],float(z['yaw_world_rad'][400]),np.asarray(state['position'])[:2],yaw)
    pyaw=z['yaw_world_rad'][400:]-z['yaw_world_rad'][400]+yaw
    np.savez(out/'prediction_before_chain.npz',position_world_xy=prediction,yaw_world_rad=pyaw)
    write(out/'prediction_status.json',{'built_at_block':160,'source':str(source),'hypothesis':'rigid relocation only, not eligible under old exact-state gate'})
  target=np.array([1.5,angles[0 if block<80 or 120<=block<160 else 1]],dtype=np.float32)
  action=np.tile(target,(protocol.controls_per_block,1)).reshape(-1)
  step=runtime.execute(action)
 np.savez(out/'physics.npz',**runtime.physics_diagnostics)
 write(out/'status.json',{'complete':True,'blocks':200,'physics_samples':len(runtime.physics_diagnostics['simulation_time_s'])})
 code=0
except BaseException:
 import traceback
 write(out/'failure.json',{'traceback':traceback.format_exc()})
 raise
finally:runtime.close(return_code=code)
