import argparse,math,json
from pathlib import Path
import numpy as np
from helpers import write,snapshot,EventOracle
from benchmark.policies import EpisodeContext
from benchmark.drive_model import DriveModel
from training.action_protocol import load_action_protocol
from scripts.run_round4_fixed_reference import load_transformed_reference
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
from template import replay_prediction
p=argparse.ArgumentParser();p.add_argument('--kind',choices=['LR','RL']);p.add_argument('--repeat',action='store_true');a=p.parse_args()
HERE=Path(__file__).resolve().parent;out=HERE/(a.kind+('_repeat' if a.repeat else '_measure'));out.mkdir(exist_ok=False)
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
 write(out/'config.json',{'kind':a.kind,'repeat':a.repeat,'main':1.5,'angles':angles,'boundary_s':8.,'transition_s':4.,'control_scope':'prescribed open-loop commands; EventOracle never act-called'})
 for block in range(120):
  if block==80:
   state=snapshot(runtime,policy,step);state.pop('policy');write(out/'boundary.json',state)
   if a.repeat:
    expected=json.loads((HERE/(a.kind+'_measure')/'boundary.json').read_text())
    current=json.loads((out/'boundary.json').read_text())
    template=np.load(HERE/(a.kind+'_measure')/'physics.npz')['position_world_xy'][400:]
    try:
     prediction=replay_prediction(expected,current,template)
     np.savez(out/'prediction_before_transition.npz',position_world_xy=prediction)
     write(out/'prediction_status.json',{'eligible':True,'built_at_block':80,'source':a.kind+'_measure'})
    except ValueError:
     write(out/'prediction_status.json',{'eligible':False,'reason':'full start mismatch','built_at_block':80})
  target=np.array([1.5,angles[0 if block<80 else 1]],dtype=np.float32)
  action=np.tile(target,(protocol.controls_per_block,1)).reshape(-1)
  step=runtime.execute(action)
 np.savez(out/'physics.npz',**runtime.physics_diagnostics)
 write(out/'status.json',{'complete':True,'blocks':120,'physics_samples':len(runtime.physics_diagnostics['simulation_time_s'])})
 code=0
except BaseException:
 import traceback
 write(out/'failure.json',{'traceback':traceback.format_exc()})
 raise
finally:runtime.close(return_code=code)
