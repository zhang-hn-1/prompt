"""Author actual-source reconstruction; no simulator/action/model fitting."""
from pathlib import Path
import sys,json,hashlib,datetime
import numpy as np
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P));O=H/'oracle_rgb_shadow_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
from oracle_rgb_shadow import OnlineRGBShadow,make_oracle,reference_from_arrays,replay_shadow
from benchmark.policies import PolicyObservation,EpisodeContext
from training.action_protocol import load_action_protocol
class FutureTrap(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('future payload')
  return super().__getitem__(k)
def main():
 p=json.loads((H/'oracle_rgb_shadow_protocol.json').read_text());a=json.loads((H/'oracle_rgb_shadow_assessment.json').read_text());m=json.loads((O/'artifact_manifest.json').read_text());status=json.loads((H/'oracle_rgb_shadow_status.json').read_text())
 assert p['sealed_utc']<status['start_utc']<m['saved_utc']<a['assessed_utc'] and status['exit_code']==0
 for f,s in p['source_sha256'].items():assert sha(f)==s,f
 for f,s in m['artifact_sha256'].items():assert sha(f)==s,f
 data=np.load(H/'oracle_rgb_shadow_reference.npz');old=np.load(H.parent/'oracle_feasible_route_20260929_v9/navigation_hybrid_golden_golden_e1_0001/episode_golden_e1_0001/states.npz')
 for k,oldk in [('pose','kinodynamic_reference_pose'),('direction','kinodynamic_reference_drive_direction'),('command','kinodynamic_reference_command')]:np.testing.assert_equal(data[k],old[oldk])
 plan=reference_from_arrays(data['pose'],data['direction'],data['command'],8.7,483)
 protocol=load_action_protocol(P/'configs/data_contract.yaml');off=make_oracle(protocol);on=make_oracle(protocol);context=EpisodeContext('golden_e1_0001',True,np.load(O/'oracle_path.npz')['xy'],protocol.protocol_id,plan.waypoints);off.reset(context);on.reset(context)
 masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');shadow=OnlineRGBShadow({eye:masks[eye] for eye in ['left','right']});packets=[];records=json.loads((O/'policy_records.json').read_text());identity=json.loads((O/'identity.json').read_text());episode=np.load(O/'episode_arrays.npz');N=identity['captures'];actions=0
 for i in range(N):
  rgb=np.load(O/'online_inputs'/f'frame_{i:03d}.npz');poses=np.load(O/'known_poses'/f'frame_{i:03d}.npz');assert set(rgb.files)=={'left_rgb','right_rgb','time_s'} and set(poses.files)=={'left_camera_usd_world','right_camera_usd_world','time_s'}
  t=float(rgb['time_s']);assert t==float(poses['time_s']);packet=dict(frame=i,time_s=t,**{k:rgb[k] for k in ['left_rgb','right_rgb']},**{k:poses[k] for k in ['left_camera_usd_world','right_camera_usd_world']});packets.append(packet)
  r=shadow.observe(packet);saved=json.loads((O/'shadow'/f'prediction_{i:03d}.json').read_text())
  for k in ['current','causal','source_views','records','floor_cells','hazard_cells','certified_free','source_model_admitted','action_authorized','pose_sigma_m']:assert r[k]==saved[k],(i,k)
  features=np.load(O/'shadow'/f'features_{i:03d}.npz');grid=np.load(O/'shadow'/f'map_{i:03d}.npz')
  for k,v in shadow.latest['features'].items():np.testing.assert_equal(v,features[k])
  for k in ['labels','provenance','floor_time','hazard_time']:np.testing.assert_equal(shadow.latest['map'][k],grid[k])
  assert saved['shadow_serialized_before_next_action'] and not saved['source_model_admitted'] and not saved['action_authorized']
  if i<len(records):
   inp=np.load(O/'policy_inputs'/f'frame_{i:03d}.npz');obs=PolicyObservation(inp['left_rgb'],inp['right_rgb'],inp['proprio_history'],inp['action_history'],inp['position_world_xy'],float(inp['yaw_world_rad']),inp['quaternion_history']);goal=inp['pointgoal_base_xy']
   x=off.act(obs,goal);y=on.act(obs,goal);np.testing.assert_equal(x,y);np.testing.assert_equal(x,inp['request']);np.testing.assert_equal(x,episode['actions'][i]);assert off.diagnostics==on.diagnostics==records[i]['diagnostics'];assert off.request_diagnostics==on.request_diagnostics==records[i]['request'];assert off.mapping_records[-1]==on.mapping_records[-1]==records[i]['mapping'];actions+=1
  if i%60==0:print('AUTHOR_REPLAY',i,flush=True)
 # Three bounded real RGB prefix substitutions: cold-start, mid-episode, final.
 future_checks=0
 for i in [0,N//2,N-1]:
  r=replay_shadow(packets[:i+1]+[FutureTrap(time_s=packets[i]['time_s']+1.)],packets[i]['time_s'],masks={eye:masks[eye] for eye in ['left','right']});s=json.loads((O/'shadow'/f'prediction_{i:03d}.json').read_text())
  for k in ['current','causal','source_views','records','floor_cells','hazard_cells']:assert r[k]==s[k],(i,k)
  future_checks+=1
 assert N==actions+1 and not a['source_admitted'] and not a['consumer_admitted'];assert a['acquisition']['physics_samples']==5*actions+1
 clocks=json.loads((O/'camera_clocks.json').read_text());assert max(abs(t-x['raw_time_s']) for x in clocks for t in x['eyes'].values())<=1e-5
 markers=json.loads((O/'marker_checks.json').read_text());assert all(v['computed_visibility']=='invisible' for row in markers for v in row.values())
 files=[f for f in O.rglob('*') if f.is_file()]+[f for f in H.glob('*oracle_rgb_shadow*') if f.is_file() and f.name!='oracle_rgb_shadow_verification.json' and not f.name.startswith('oracle_rgb_shadow_verify')]+[H.parent.parent/'reports/benchmark_oracle_online_rgb_shadow_20261005.md']
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sealed_source_checks=len(p['source_sha256']),complete_output_checks=len(m['artifact_sha256']),cached_reference_exact=True,full_captures_reconstructed=N,full_features_and_map_parity=N,actual_requests_policy_on_off_exact=actions,real_rgb_future_poison_truncation_checks=future_checks,original_budget_s=35.,actual_elapsed_s=a['navigation']['elapsed_s'],source_admitted=False,consumer_admitted=False,whole_research_complete=False,source_and_artifact_sha256={str(f):sha(f) for f in files if f.exists()})
 (H/'oracle_rgb_shadow_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='source_and_artifact_sha256'},indent=2))
if __name__=='__main__':main()
