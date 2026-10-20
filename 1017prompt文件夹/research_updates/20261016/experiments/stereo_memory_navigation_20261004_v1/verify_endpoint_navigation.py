"""Author actual-data reconstruction, no simulator/fit or alternative controller."""
from pathlib import Path
import sys,json,hashlib,datetime,copy
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P));O=H/'endpoint_navigation_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
from outgoing_lookahead import EndpointLookaheadPolicy
from depth_local_curve import causal_packets
from benchmark.policies import PolicyObservation,EpisodeContext
from training.action_protocol import load_action_protocol
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
def untimed(value):
 if isinstance(value,dict):return {k:untimed(v) for k,v in value.items() if k!='seconds'}
 if isinstance(value,list):return [untimed(v) for v in value]
 return value
class FutureTrap(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('future payload read')
  return super().__getitem__(k)
def main():
 p=json.loads((H/'endpoint_navigation_protocol.json').read_text());m=json.loads((O/'artifact_manifest_post_close.json').read_text());a=json.loads((H/'endpoint_navigation_assessment.json').read_text());status=json.loads((H/'endpoint_navigation_status.json').read_text());assert status['exit_code']==0 and p['sealed_utc']<status['start_utc']<m['saved_utc']<a['assessed_utc']
 for f,s in p['source_sha256'].items():assert sha(f)==s,f
 for f,s in p['input_sha256'].items():assert sha(f)==s,f
 for f,s in m['artifact_sha256'].items():assert sha(f)==s,f
 records=json.loads((O/'policy_records.json').read_text());N=len(records);arrays=np.load(O/'episode_arrays.npz');ph=np.load(O/'physics.npz');states=json.loads((O/'execution_states.json').read_text());protocol=load_action_protocol(P/'configs/data_contract.yaml');policy=EndpointLookaheadPolicy(protocol);policy.reset(EpisodeContext('golden_e1_0001',False,None,protocol.protocol_id));futurechecks=0;previous=None
 masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');yy,xx=np.mgrid[0:480:4,0:640:4];rays=np.c_[(xx.ravel()-319.5)/584.8888888888889,(yy.ravel()-239.5)/584.8888888888889,np.ones(xx.size)]
 for i in range(N+1):
  sensors=np.load(O/'sensor_packets'/f'frame_{i:03d}.npz');packet={k:sensors[k] for k in sensors.files};raw=np.load(O/'offline_truth'/f'frame_{i:03d}.npz');points=[]
  assert set(packet)=={'frame','time_s','points_world','root_xyz'} and int(packet['frame'])==i
  for eye in ['left','right']:
   d=raw[eye+'_depth_m'][::4,::4].ravel();pose=raw[eye+'_camera_usd_world'];valid=np.isfinite(d)&(d>=.45)&(d<=8.)&(~masks[eye][::4,::4].ravel());rotation=np.c_[pose[:3,0],-pose[:3,1],-pose[:3,2]];points.append(pose[:3,3]+(rays[valid]*d[valid,None])@rotation.T)
  np.testing.assert_equal(packet['points_world'],np.concatenate(points));np.testing.assert_equal(packet['root_xyz'],raw['root_position_world_m']);assert float(packet['time_s'])==float(raw['time_s'])
  hist=np.load(O/'history'/f'frame_{i:03d}.npz')
  if previous is not None:
   for k in hist.files:np.testing.assert_equal(hist[k][:-1],previous[k][1:])
  previous={k:hist[k].copy() for k in hist.files}
  if i==N:continue
  inp=np.load(O/'policy_inputs'/f'frame_{i:03d}.npz');obs=PolicyObservation(inp['left_rgb'],inp['right_rgb'],inp['proprio_history'],inp['action_history'],inp['position_world_xy'],float(inp['yaw_world_rad']),inp['quaternion_history']);goal=inp['pointgoal_base_xy'];check=copy.deepcopy(policy) if i in [0,N//2,N-1] else None
  policy.set_current_packet(packet);request=policy.act(obs,goal);np.testing.assert_equal(request,inp['request']);np.testing.assert_equal(request,arrays['actions'][i]);r=records[i]
  assert untimed(policy.records[-1])==untimed(r['local']),(i,'local')
  assert policy.diagnostics==r['diagnostics'] and policy.request_diagnostics==r['request'] and policy.tracking_diagnostics==r['tracking'] and policy.tracker.mapping_records[-1]==r['mapping'],(i,'policy')
  saved=np.load(O/'maps'/f'frame_{i:03d}.npz')
  for k,v in [('labels',policy.grid.labels),('provenance',policy.grid.provenance()),('floor_time',policy.grid.memory.floor_time),('hazard_time',policy.grid.memory.hazard_time)]:np.testing.assert_equal(v,saved[k])
  assert not policy.records[-1]['global_reference_consumed'] and policy.records[-1]['body_prior_observed_free']==0
  if policy.plan is not None:
   ref=np.load(O/'local_plans'/f'frame_{i:03d}.npz');way=policy.plan.waypoints
   np.testing.assert_equal(ref['pose'],[[w.x_m,w.y_m,w.yaw_rad] for w in way]);np.testing.assert_equal(ref['direction'],[1 if w.drive_direction=='forward' else -1 for w in way]);np.testing.assert_equal(ref['command'],[[w.main_command_rad_s,w.auxiliary_angle_rad] for w in way]);assert float(ref['time_s'])==policy.reference_installed_time
  if check is not None:
   causal=list(causal_packets([packet,FutureTrap(time_s=float(packet['time_s'])+.1)],packet['time_s']));assert len(causal)==1
   check.set_current_packet(causal[0]);other=check.act(obs,goal);np.testing.assert_equal(other,request);assert check.diagnostics==policy.diagnostics and untimed(check.records[-1])==untimed(policy.records[-1]);np.testing.assert_equal(check.grid.labels,policy.grid.labels);futurechecks+=1
  if i%60==0:print('AUTHOR_ENDPOINT_NAVIGATION_REPLAY',i,flush=True)
 # Every physics request matches actual act; deterministic execution uses measured current feedback only.
 np.testing.assert_equal(ph['raw_command_target'][1:],arrays['actions'].reshape(-1,2));assert states[0] is None and len(states)==N+1
 clone=ExecutionClone.from_snapshot(states[1]);ticks=0
 for tick in range(5,5*N):
  applied,smooth=clone.step(ph['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][tick]),aux_position_rad=ph['joint_position_rad'][tick,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][tick,1]);np.testing.assert_equal(applied,ph['applied_action'][tick+1]);np.testing.assert_equal(smooth,ph['smoothed_target'][tick+1])
  if (tick+1)%5==0:assert clone.snapshot()==states[(tick+1)//5],tick
  ticks+=1
 clocks=json.loads((O/'camera_clocks.json').read_text());markers=json.loads((O/'marker_checks.json').read_text());assert len(clocks)==N+1 and max(abs(x['raw_time_s']-v) for x in clocks for v in x['eyes'].values())<=1e-5;assert all(v['computed_visibility']=='invisible' for row in markers for v in row.values())
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_checks=len(p['source_sha256']),post_close_artifact_checks=len(m['artifact_sha256']),actual_policy_requests_reconstructed=N,all_maps_and_local_references_exact=True,bilateral_depth_packet_reconstruction=N+1,history_shifts=N,finite_past_state_checkpoint_future_payload_truncation_checks=futurechecks,execution_measured_feedback_reconstructed_ticks=ticks,execution_initial_5_ticks_not_cloned=True,physical_feedback_not_recursive_prediction=True,zero_camera_age=True,hidden_markers=True,navigation_gate_pass=a['navigation_gate_pass'],interface_gate_pass=True,source_rgb_gate_unchanged_rejected=True,whole_research_complete=False)
 (H/'endpoint_navigation_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
