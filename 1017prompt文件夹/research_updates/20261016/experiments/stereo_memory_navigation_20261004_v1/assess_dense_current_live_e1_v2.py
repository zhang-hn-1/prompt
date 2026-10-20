"""Complete independent measured navigation evaluation; offline provenance replay.

Source replay verifies already saved current packets. It is not an online
validation, new CAD query, teacher experiment or learned model admission.
"""
from pathlib import Path
import hashlib,json,numpy as np,cv2
from baseline35_metrics_v1 import assess,observation_gap_diagnostics
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence,cross_metrics
from dense_current_live_gates_v1 import gates
from dense_current_live_termination_v1 import validate_termination
from dense_source_points import point_packet
from dense_current_source_packet_v1 import dense_point_packet
from paired_texture_surface import PairedTextureSurfaceMemory
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def verify(table):
 for p,d in table.items():assert sha(p)==d,p

def source_replay(d,rows,variant):
 surface=PairedTextureSurfaceMemory();count=[]
 sample=dense_point_packet if variant=='candidate' else point_packet
 for row in rows:
  frame=row['frame'];t=row['time_s']
  raw=dict(np.load(d/'source_predictions'/f'frame_{frame:03d}.npz'))
  pose=dict(np.load(d/'known_poses'/f'frame_{frame:03d}.npz'))
  masks=dict(np.load(d/'current_masks'/f'frame_{frame:03d}.npz'))
  history=np.load(d/'history'/f'frame_{frame:03d}.npz')['proprio_history']
  packet=dict(np.load(d/'sensor_packets'/f'source_{frame:03d}.npz'))
  assert all(float(x['time_s'])==t for x in (raw,pose,masks,packet))
  assert int(raw['frame'])==int(masks['frame'])==int(packet['frame'])==frame
  np.testing.assert_array_equal(masks['joints_current'],history[-1,9:12])
  prediction=dict(depth_m=raw['depth_m'],confidence=raw['confidence'],disparity_native_px=restore_native(raw['disparity_px']),frame=frame,time_s=t)
  expected=sample(prediction,pose,masks['left'],masks['right'])
  for k in ('world','sigma_m','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(packet[k],expected[k])
  assert not bool(packet['certified_free']) and not bool(packet['source_model_admitted'])
  verify(row['provenance'])
  rgb=dict(np.load(d/'source_inputs'/f'frame_{frame:03d}.npz'));assert float(rgb['time_s'])==t
  measured=surface.update(packet,rgb['left_rgb'],rgb['right_rgb'],t,row['provenance'],prediction['disparity_native_px'])
  saved=dict(np.load(d/'partial_observations'/f'frame_{frame:03d}.npz'))
  for k in ('world','sigma_m','query_pixel_id','partner_pixel_id'):np.testing.assert_array_equal(saved[k],measured[k])
  assert row['source_packet_points']==len(packet['world']) and row['points']==len(saved['world'])
  assert row['live_packet_sampler']==('without_stride2' if variant=='candidate' else 'original_stride2')
  assert row['actual_source_transport_module']==('dense_current_live_transport_v1' if variant=='candidate' else 'masked_source_pair')
  count.append(dict(frame=frame,time_s=t,packet=len(packet['world']),partial=len(saved['world'])))
 return count

def arm(name):
 d=H/f'dense_current_live_e1_{name}_run_v1'
 cfg=json.loads((H/f'dense_current_live_e1_{name}_protocol_v2.json').read_text())
 verify(cfg['source_sha256']);verify(cfg['input_sha256'])
 status=json.loads((H/f'dense_current_live_e1_{name}_status_v1.json').read_text())
 assert status['status']=='finished' and status['child_returncode']==0 and not status['timeout'] and not status['forced_kill'] and status['parent_signal'] is None
 seal=json.loads((d/'artifact_manifest_post_close.json').read_text());assert seal['status']==status;verify(seal['artifact_sha256'])
 assert json.loads((d/'live_transport_contract.json').read_text())==dict(arm=name,protocol=cfg)
 identity=json.loads((d/'identity.json').read_text());assert identity['complete'] and identity['known_GT_pose_control'] and identity['source_GT_pose_registration'] and identity['reset_count']==1 and not identity['internal_reset']
 assert identity['live_packet_sampler']==('without_stride2' if name=='candidate' else 'original_stride2')
 states=json.loads((d/'states.json').read_text());contacts=json.loads((d/'physics_contacts.json').read_text());physics=dict(np.load(d/'physics.npz'));requests=np.load(d/'requests.npz')['requests'];rows=json.loads((d/'policy_records.json').read_text())
 assert len(requests)==len(rows)==identity['actions'] and len(states)==identity['captures']
 np.testing.assert_array_equal(requests,np.asarray([r['request'] for r in rows],np.float32))
 np.testing.assert_array_equal(physics['raw_command_target'][1:],requests.reshape(-1,2))
 normal=complete_execution_evidence(states,contacts,physics['simulation_time_s'],len(requests))
 episode=json.loads((d/'task_episode.json').read_text())['actual_episode'];start=np.asarray(episode['start_position_world_m'][:2]);goal=np.asarray(episode['goal_position_world_m'][:2])
 m=assess(states,normal,start,goal);cross=cross_metrics(states,start,goal,m['scored_end_s']);m.update(cross_rms_m=cross['rms_m'],cross_peak_m=cross['peak_m'])
 m['execution_stop_class']=validate_termination(json.loads((d/'termination.json').read_text()),m,identity,len(requests))
 assert all(r['known_GT_pose'] and not r['native_pose_used'] and r.get('measured_free',0)==0 for r in rows)
 # Inspect all saved map semantics; changing positive source sampling never
 # permits writing unknown or prior cells as observed free.
 for r in rows:
  mp=np.load(d/'maps'/f"frame_{r['frame']:03d}.npz")
  assert not (mp['labels']==0).any() and set(np.unique(mp['labels'])).issubset({-1,1})
 videos={}
 for n in ('stereo_view','third_person'):
  video=cv2.VideoCapture(str(d/(n+'.mp4')));fps=video.get(cv2.CAP_PROP_FPS);length=0
  while video.read()[0]:length+=1
  video.release();assert fps==10. and length==len(states);videos[n]=dict(frames=length,fps=fps)
 clocks=json.loads((d/'camera_clocks.json').read_text());assert len(clocks)==len(states)
 assert all(abs(c['eyes'][e]-c['raw_time_s'])<=1e-7 for c in clocks for e in ('left','right'))
 source=json.loads((d/'source_rows.json').read_text());assert [r['frame'] for r in source]==list(range(0,len(states),5))
 verified=source_replay(d,source,name)
 wall=np.asarray([r['end_to_end_wall_s'] for r in rows]);assert len(wall)>0 and np.isfinite(wall).all()
 m.update(raw_contacts_zero=not any(c['contact'] for c in contacts),wall_median_s=float(np.median(wall)),wall_p90_s=float(np.percentile(wall,90)),source_gaps=observation_gap_diagnostics(source,rows),source_packet_contract_verified=verified,videos=videos,requests=len(requests),captures=len(states),physics_samples=len(physics['simulation_time_s']),contact_samples=len(contacts),sealed_items=len(seal['artifact_sha256']))
 return m,dict(d=d,states=states,physics=physics,requests=requests,identity=identity,episode=episode)

def main():
 a,ar=arm('b0');b,br=arm('candidate');assert ar['episode']==br['episode']
 for k in ('camera_rig','pid_config','inner_config','policy_MRO','tracker_MRO','frozen_source_method','source_sha256','input_sha256'):assert ar['identity'][k]==br['identity'][k],k
 n=min(len(ar['requests']),len(br['requests']));indices=np.flatnonzero(np.any(ar['requests'][:n]!=br['requests'][:n],axis=1));fork=int(indices[0]) if len(indices) else None
 before=fork*.1 if fork is not None else min(ar['states'][-1]['time_s'],br['states'][-1]['time_s'])
 ap=ar['physics'];bp=br['physics'];ma=ap['simulation_time_s']<=before+1e-6;mb=bp['simulation_time_s']<=before+1e-6
 prefix=all(np.array_equal(ap[k][ma],bp[k][mb]) for k in ap if not k.startswith('raw_'))
 prefix_images=[]
 for f in range(0,int(round(before*10))+1,5):
  x=dict(np.load(ar['d']/'source_inputs'/f'frame_{f:03d}.npz'));y=dict(np.load(br['d']/'source_inputs'/f'frame_{f:03d}.npz'))
  prefix_images.append(dict(frame=f,RGB_exact=all(np.array_equal(x[k],y[k]) for k in ('left_rgb','right_rgb'))))
 g=gates(a,b,changed=fork is not None,contract=True)
 out=dict(arms=dict(b0=a,candidate=b),gates=g,TaskH_development_navigation_accept=all(g.values()),classification='measured_navigation_accept' if all(g.values()) else ('measured_initial_view_handoff_reject' if b['execution_stop_class']=='initial_view_handoff_reject' else ('measured_navigation_reject' if fork is not None else 'unassessed_incremental_execution_effect')),first_actual_request_fork=fork,prefix_physics_exact=prefix,prefix_source_RGB=prefix_images,full_causal_claim=False,within_arm_shadow_not_cross_arm_counterfactual=True,source_accuracy_admitted=False,free_volume_certified=False,strict_visual_admission=False,estimated_pose_result=None,no_new_teacher_or_fit=True,whole_mission_complete=False)
 p=H/'dense_current_live_e1_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k!='arms'}))
if __name__=='__main__':main()
