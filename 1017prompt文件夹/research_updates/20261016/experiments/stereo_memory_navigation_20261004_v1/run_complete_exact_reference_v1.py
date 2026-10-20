"""One whole fixed-reference diagnostic. No depth model, planner or learned change."""
from pathlib import Path
import sys,json,dataclasses,hashlib,datetime,numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
import scripts.run_pointgoal_benchmark as runner
from benchmark.runner import run_episode,RunnerConfig
from benchmark.artifacts import write_rgb_video
from benchmark.policies import PolicyObservation
from scipy.spatial.transform import Rotation
from outgoing_lookahead import make_endpoint_oracle
from exact_outgoing_lookahead import with_exact_outgoing_lookahead
from depth_local_curve import install_local_reference
from oracle_rgb_shadow import reference_from_arrays
from estimated_navigation_observation import EstimatedNavigationObservation
from execution_snapshot import export_execution
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from roll_pid import install_runtime
D=H/'original_E1_repeated_exposure_memory_navigation_run_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def loadplan(frame):
 d=json.loads((D/'local_plans'/f'frame_{frame:03d}.json').read_text());w=d['waypoints'];return reference_from_arrays([[v[k] for k in ['x_m','y_m','yaw_rad']] for v in w],[1 if v['drive_direction']=='forward' else -1 for v in w],[[v['main_command_rad_s'],v['auxiliary_angle_rad']] for v in w],d['path_length_m'],d['expanded_nodes'])
def main():
 cfg=json.loads((H/'complete_exact_reference_protocol_v1.json').read_text());variant=sys.argv[1];assert variant in ['B0','exact'];assert all(sha(f)==v for f,v in cfg['source_sha256'].items());out=H/f'complete_exact_reference_{variant}_run_v1';out.mkdir(exist_ok=False)
 original=next(e for e in runner.load_manifest(cfg['manifest']).episodes if e.episode_id=='golden_e1_0001');end=loadplan(155).waypoints[-1];episode=dataclasses.replace(original,episode_id='complete_fixed_reference_'+variant,goal_position_world_m=(end.x_m,end.y_m,.45),euclidean_distance_m=float(np.linalg.norm(np.array([end.x_m,end.y_m])-np.array(original.start_position_world_m[:2]))));occupancy=runner._resolve_occupancy_path(P/'configs/benchmark_v1/scene_index.json',original,P/'data');protocol=runner.load_action_protocol(P/'configs/data_contract.yaml');frozen=np.load(D/'requests.npz')['requests'];nav=EstimatedNavigationObservation();states=[];execution=[];records=[];contacts=[]
 class RuntimeCapture(runner.IsaacBenchmarkRuntime):
  def _capture(self):
   step=super()._capture();p,q=self._robot.get_world_pose();v=self._robot.get_linear_velocity();av=self._robot.get_angular_velocity();R=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix();states.append(dict(time_s=float(step.simulation_time_s),position=np.asarray(p).tolist(),quaternion=np.asarray(q).tolist(),velocity=np.asarray(v).tolist(),angular_velocity=np.asarray(av).tolist(),joints=np.asarray(self._robot.get_joint_positions()).tolist(),joint_velocity=np.asarray(self._robot.get_joint_velocities()).tolist()));execution.append(export_execution(self._action_smoother,self._action_protocol) if hasattr(self._action_smoother,"pid") else None);contacts.append(dict(time_s=float(step.simulation_time_s),collision=bool(step.collision),physical_contact=bool(step.physical_contact)))
   obs,goal=nav.build(step.observation,dict(time_s=float(step.simulation_time_s),root_position_world_m=p,root_rotation_world=R,velocity_world_m_s=v),R.T@av,original.goal_position_world_m if len(states)-1<155 else episode.goal_position_world_m);return dataclasses.replace(step,observation=obs)
 gains=json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/selected_candidate.json').read_text())['gains'];Runtime=install_runtime(RuntimeCapture,**gains,output=out/'feedback');runtime=Runtime(runner.IsaacRuntimeConfig(scene_root=Path('/home/an/project/NavDP/assets/scenes'),contract_path=P/'configs/data_contract.yaml',occupancy_path=occupancy,record_video=True,gui=False,diagnostic_physics_trace=True,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2',auxiliary_velocity_limit_rad_s=.3))
 class Policy:
  policy_name='complete_cached_reference_single_geometry_diagnostic';privileged=False;uses_shared_tracker=True
  def reset(self,context):
   self.tracker=make_endpoint_oracle(protocol)
   if variant=='exact':self.tracker=with_exact_outgoing_lookahead(type(self.tracker))(protocol,lookahead_m=.5)
   self.tracker.mapping_records=[];self.i=0
  @property
  def diagnostics(self):return self.tracker.diagnostics
  @property
  def request_diagnostics(self):return self.tracker.request_diagnostics
  @property
  def tracking_diagnostics(self):return self.tracker.tracking_diagnostics
  def act(self,obs,goal):
   i=self.i
   if i in [60,155]:install_local_reference(self.tracker,loadplan(i))
   tracker_request=None
   if i>=60:
    target=np.asarray(original.goal_position_world_m[:2] if i<155 else episode.goal_position_world_m[:2]);delta=target-obs.position_world_xy;cy,sy=np.cos(obs.yaw_world_rad),np.sin(obs.yaw_world_rad);tg=np.array([[cy,sy],[-sy,cy]])@delta;tracker_request=self.tracker.act(obs,tg).copy();tracker_request[::2]=np.clip(tracker_request[::2],-.3,.5)
   action=frozen[i].copy() if i<155 else tracker_request
   records.append(dict(frame=i,time_s=states[-1]['time_s'],request=action.tolist(),tracker_request=None if tracker_request is None else tracker_request.tolist(),prefix_frozen=i<155,tracking=self.tracker.tracking_diagnostics if i>=60 else {},state=self.tracker.diagnostics,geometry_candidate=variant=='exact'));self.i+=1
   if i%50==0:print('FRAME',i,states[-1]['time_s'],flush=True)
   return action
 result=None;code=1
 try:
  result=run_episode(runtime,Policy(),episode,action_protocol=protocol,config=RunnerConfig(episode_budget_s=75.,diagnostic_settling_budget_s=3.,diagnostic_stop_hold_s=3.1));np.savez_compressed(out/'episode_arrays.npz',**{f.name:getattr(result,f.name) for f in dataclasses.fields(result) if isinstance(getattr(result,f.name),np.ndarray)});(out/'metrics.json').write_text(json.dumps(result.to_metrics_dict(),indent=2)+'\n');code=0 if result.runtime_error is None and result.diagnostic_settling_runtime_error is None else 1;print('COMPLETE',variant,result.termination_reason,result.elapsed_s,result.final_distance_m,flush=True)
 finally:
  np.savez_compressed(out/'physics.npz',**runtime.physics_diagnostics)
  for name,frames in runtime.captured_frames.items():write_rgb_video(out/(name+'.mp4'),frames,fps=10.)
  for n,data in [('states',states),('execution_states',execution),('policy_records',records),('contacts',contacts),('physics_contacts',runtime.collision_contact_samples)]: (out/(n+'.json')).write_text(json.dumps(data,indent=2)+'\n')
  (out/'identity.json').write_text(json.dumps(dict(variant=variant,completed=code==0,original_episode=original.to_dict(),actual_episode=episode.to_dict(),reset_count=1,internal_reset=False,cached_complete_reference_prior=True,not_visual_navigation=True,not_original35=True,known_pose_diagnostic=True,controller_source_sha256=cfg['source_sha256'],formal35_NOTRUN=True,whole_research_complete=False),indent=2)+'\n');runtime.close(return_code=code)
if __name__=='__main__':main()
