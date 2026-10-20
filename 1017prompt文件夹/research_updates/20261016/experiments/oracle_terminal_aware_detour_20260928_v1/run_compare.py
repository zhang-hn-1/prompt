import argparse,math,json,dataclasses
from pathlib import Path
import numpy as np
from helpers import write,snapshot,EventOracle
from benchmark.policies import EpisodeContext
from benchmark.drive_model import DriveModel
from training.action_protocol import load_action_protocol
from scripts.run_round4_fixed_reference import load_transformed_reference
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
from geometry import relocate
from gate import clearance
from rejoin import rejoin_path
from selector import select_sequence
from benchmark.kinodynamic_planning import KinodynamicWaypoint
p=argparse.ArgumentParser();p.add_argument('--kind',choices=['LR','RL']);p.add_argument('--mode',choices=['baseline','candidate']);a=p.parse_args()
HERE=Path(__file__).resolve().parent;out=HERE/(a.kind+'_'+a.mode);out.mkdir(exist_ok=False)
ref,_=load_transformed_reference(HERE.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
episode=build_tracking_episode(ref.case,ref,scene_root=HERE/'scenes',fixture_path=HERE/'scenes'/f'{a.kind}_blocked.usda',occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
protocol=load_action_protocol(DEFAULT_CONTRACT)
class ContactRuntime(IsaacBenchmarkRuntime):
 def _record_physics_diagnostic(self,*args,**kwargs):
  super()._record_physics_diagnostic(*args,**kwargs)
  if not hasattr(self,'contact_trace'):self.contact_trace=[]
  contact=self._physical_contact()
  self.contact_trace.append([bool(contact),bool(self._contact_available),self._contacts is not None])
runtime=ContactRuntime(IsaacRuntimeConfig(scene_root=(HERE/'scenes').resolve(),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,record_video=False,occupancy_path=DEFAULT_OCCUPANCY.resolve(),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),diagnostic_open_plane=True,diagnostic_physics_trace=True,diagnostic_auxiliary_target_rate_rad_s=math.radians(30),diagnostic_auxiliary_velocity_limit_rad_s=.30))
policy=EventOracle(protocol,drive_model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE));policy.reset(EpisodeContext(episode_id=episode.episode_id,privileged=True,oracle_path_xy=ref.points_xy,action_protocol_id=protocol.protocol_id,kinodynamic_reference=ref.waypoints))
angles=np.radians([18.425763799461098,-24.4231088098262]);angles=angles if a.kind=='LR' else angles[::-1]
runtime.set_reference_visualization_path(ref.points_xy)
manifest=json.loads((HERE/'manifest.json').read_text())[a.kind]
obstacle=manifest['obstacle'];goal=np.array(manifest['goal']);forward=np.array(manifest['forward'])
episode=dataclasses.replace(episode,goal_position_world_m=(*goal,.45))
selected=None;tracking=False;goal_hit=False;switch_block=999;rejoin_block=999
code=1
try:
 step=runtime.reset(episode)
 write(out/'config.json',{'kind':a.kind,'main':1.5,'angles':angles,'boundary_s':8.,'transition_s':12.,'control_scope':'frozen compound selection at16s; unchanged Oracle after selected sequence','mode':a.mode,'obstacle':obstacle,'goal':goal})
 for block in range(350):
  if block in [80,120,160,rejoin_block]:
   state=snapshot(runtime,policy,step);state.pop('policy');write(out/f'boundary_{block}.json',state)
   if block==160:
    current=json.loads((out/'boundary_160.json').read_text())
    expected=json.loads((HERE.parent/'oracle_local_detour_20260928_v1'/f'{a.kind}_hold/boundary_160.json').read_text())
    eligible=current==expected
    q=state['quaternion'];yaw=math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2));choices={}
    hold=np.load(HERE.parent/'oracle_local_detour_20260928_v1'/f'{a.kind}_hold/physics.npz')
    turn=np.load(HERE.parent/'oracle_measured_turn_primitives_20260928_v1'/f'{a.kind}_measure/physics.npz')
    chosen,candidates=select_sequence(hold,turn,np.asarray(state['position'])[:2],yaw,goal,obstacle['center_xy'],19.)
    if not eligible:chosen=None
    selected='compound' if chosen is not None else 'brake'
    if chosen is not None:
     switch_block=160+int(round(chosen['hold_s']*10));rejoin_block=switch_block+40
     np.savez(out/'compound_prediction_before_action.npz',position_world_xy=chosen['xy'])
    write(out/'selection.json',{'eligible_exact_collected_start':eligible,'selected':selected,'chosen':chosen,'candidates':candidates,'built_at_block':160,'switch_block':switch_block,'rejoin_block':rejoin_block})
   if block==rejoin_block and a.mode=='candidate' and selected!='brake':
    path=rejoin_path(step.observation.position_world_xy,goal,np.array(obstacle['center_xy']),forward)
    if path is not None:
     delta=np.diff(path,axis=0);headings=np.arctan2(delta[:,1],delta[:,0]);headings=np.r_[headings,headings[-1]]
     waypoints=tuple(KinodynamicWaypoint(float(x),float(y),float(yaw),'forward',1.5,0.) for (x,y),yaw in zip(path,headings))
     policy=EventOracle(protocol,lookahead_m=.5,yaw_rate_feedback_gain_s=.15,drive_model=DriveModel.from_json(DEFAULT_DRIVE_PROFILE))
     policy.reset(EpisodeContext(episode_id=episode.episode_id,privileged=True,oracle_path_xy=path,action_protocol_id=protocol.protocol_id,kinodynamic_reference=waypoints));tracking=True
     write(out/'rejoin.json',{'clear':True,'path':path,'goal':goal,'constructed_at_block':block})
    else:
     selected='brake';write(out/'rejoin.json',{'clear':False,'constructed_at_block':block})
  target=np.array([1.5,angles[0 if block<80 or 120<=block<160 else 1]],dtype=np.float32)
  if block>=160:
   if selected=='brake':target=np.array([0.,angles[0]],dtype=np.float32)
   elif selected=='compound':target=np.array([1.5,angles[0 if block<switch_block else 1]],dtype=np.float32)
  if np.linalg.norm(step.observation.position_world_xy-goal)<=.2:goal_hit=True
  if goal_hit:target=np.zeros(2,dtype=np.float32)
  if tracking and not goal_hit:
   obs=step.observation;d=goal-obs.position_world_xy;c,t=math.cos(obs.yaw_world_rad),math.sin(obs.yaw_world_rad)
   action=policy.act(obs,pointgoal_base_xy=np.array([c*d[0]+t*d[1],-t*d[0]+c*d[1]]))
   step=runtime.execute(action);continue
  action=np.tile(target,(protocol.controls_per_block,1)).reshape(-1)
  step=runtime.execute(action)
 np.savez(out/'physics.npz',**runtime.physics_diagnostics)
 np.savez(out/'contact.npz',contact=runtime.contact_trace)
 write(out/'tracking.json',list(policy.tracking_records))
 write(out/'status.json',{'complete':True,'blocks':350,'physics_samples':len(runtime.physics_diagnostics['simulation_time_s'])})
 code=0
except BaseException:
 import traceback
 write(out/'failure.json',{'traceback':traceback.format_exc()})
 raise
finally:runtime.close(return_code=code)
