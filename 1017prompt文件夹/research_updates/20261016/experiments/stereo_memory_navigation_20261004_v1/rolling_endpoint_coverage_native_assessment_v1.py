"""Actual ordinary E0 pair; frozen line/current measurements, never policy cursor."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
from baseline35_metrics_v1 import assess,observation_gap_diagnostics
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence,cross_metrics
from local_reserve_execution_classification_v1 import shared_execution_prefix
from measured_speed_consumer_v1 import FEEDFORWARD,KP,TARGET,RESERVE
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def verify(mapping):
 for p,d in mapping.items():assert sha(Path(p))==d,p

def reconstruct_current_speed(velocity):
 # Native estimate is float64, formal PolicyObservation quantizes to float32; consumer then promotes.
 return float(np.linalg.norm(np.asarray(velocity,np.float32).astype(float)[:2]))

def meaningful_snapshot_equal(a,b):
 # Absolute artifact locations are audited by each arm seal, not geometric state.
 def normalized(s):
  return dict(s,records=[{k:v for k,v in r.items() if k!='input_sha256'} for r in s['records']])
 return normalized(a)==normalized(b)

def arm(name,cfg):
 D=H/(CASE+'_run_v1');status=json.loads((H/(CASE+'_status_v1.json')).read_text());assert status['status']=='finished' and status['child_returncode']==0
 seal=json.loads((D/'artifact_manifest_post_close.json').read_text());verify(seal['artifact_sha256']);assert json.loads((D/'candidate_envelope.json').read_text())==dict(arm=name,design=cfg)
 identity=json.loads((D/'identity.json').read_text());assert identity['complete'] and not identity['known_GT_pose_control'] and not identity['source_GT_pose_registration'] and identity['native_estimated_velocity_proprio_feedback'] and identity['reset_count']==1 and not identity['internal_reset']
 ss=json.loads((D/'states.json').read_text());cc=json.loads((D/'physics_contacts.json').read_text());rr=json.loads((D/'policy_records.json').read_text());q=np.load(D/'requests.npz')['requests'];physics=dict(np.load(D/'physics.npz'));episode=json.loads((D/'task_episode.json').read_text())['actual_episode'];term=json.loads((D/'termination.json').read_text())
 assert identity['actions']==len(q) and identity['captures']==len(ss) and len(q)==len(rr) and np.array_equal(q,np.asarray([r['request'] for r in rr],np.float32))
 normalized=complete_execution_evidence(ss,cc,physics['simulation_time_s'],len(q));start=np.array(episode['start_position_world_m'][:2]);goal=np.array(episode['goal_position_world_m'][:2]);m=assess(ss,normalized,start,goal)
 assert m['outcome'] in ('success','collision','timeout'),m
 assert term['termination']==dict(success='success',collision='collision',timeout='timeout35')[m['outcome']]
 assert len(q)==350 or m['outcome'] in ('success','collision')
 assert all(not r['known_GT_pose'] and r['native_pose_used'] for r in rr)
 native=json.loads((D/'native_predictions.json').read_text());packets=json.loads((D/'native_imu_packets.json').read_text())
 from rolling_native_input_audit_v1 import current_native_inputs
 for row in rr:
  ix=[i for i,r in enumerate(native) if abs(float(r['time_s'])-row['time_s'])<1e-8];assert len(ix)==1
  p,g,v=current_native_inputs(native[ix[0]],packets[ix[0]],row['time_s'])
  np.testing.assert_array_equal(p['root_position_world_m'],row['native_current_pose']['root_position_world_m'])
  np.testing.assert_array_equal(p['root_rotation_world'],row['native_current_pose']['root_rotation_world'])
  np.testing.assert_array_equal(p['velocity_world_m_s'],row['native_current_pose']['velocity_world_m_s']);assert v==row['native_worldXY_speed_m_s']
 for src in json.loads((D/'source_rows.json').read_text()):
  assert not src['current_GT_pose_registration']
  ix=[i for i,r in enumerate(native) if abs(float(r['time_s'])-src['time_s'])<1e-8];assert len(ix)==1
  p,g,v=current_native_inputs(native[ix[0]],packets[ix[0]],src['time_s'])
  saved=np.load(D/'known_poses'/f"frame_{src['frame']:03d}.npz")
  assert float(saved['time_s'])==src['time_s']
  for key in ('left_camera_usd_world','right_camera_usd_world','root_position_world_m'):np.testing.assert_array_equal(saved[key],p[key])
 for row in rr:
  if not row.get('planner_act_called'):continue
  before=np.array(row['measured_speed_before_request'],np.float32);issued=np.array(row['request'],np.float32);assert np.array_equal(before[1::2],issued[1::2]);assert row['measured_speed_control_applied']==(not np.array_equal(before,issued))
  if row['measured_speed_control_eligible']:
   speed=reconstruct_current_speed(row['native_current_pose']['velocity_world_m_s']);assert abs(speed-row['measured_speed_worldXY_m_s'])<1e-6
   assert row['measured_speed_global_distance_m']>=RESERVE
   if row.get('rolling_local_reserve_bypassed'):
    context=row['rolling_endpoint_context'];assert context['eligible'] and not context['certified_safe'] and not context['observed_free']
    assert context['local_end_to_route_end_m']>=RESERVE and context['route_end_to_current_global_goal_m']<=.2
   else:assert row['measured_speed_local_endpoint_m']>=RESERVE
   assert np.all(issued[::2]==np.float32(np.clip(FEEDFORWARD+KP*(TARGET-speed),.5,1.)))
  else:assert np.array_equal(issued,before)
  if name=='paced':
   parent=np.asarray(row['rolling_parent_request'],np.float32);assert row['rolling_control_applied']==(not np.array_equal(parent,issued))
   assert np.array_equal(parent[1::2],issued[1::2])
   if row['rolling_local_reserve_bypassed']:assert np.all(parent[::2]==.5)
   else:assert np.array_equal(parent,issued)
 videos={}
 for n in ('stereo_view','third_person'):
  v=cv2.VideoCapture(str(D/(n+'.mp4')));fps=v.get(cv2.CAP_PROP_FPS);count=0
  while v.read()[0]:count+=1
  v.release();assert fps==10. and count==len(ss);videos[n]=dict(frames=count,fps=fps)
 clocks=json.loads((D/'camera_clocks.json').read_text());assert len(clocks)==len(ss)
 for c in clocks:assert all(abs(c['eyes'][eye]-c['raw_time_s'])<1e-7 for eye in ('left','right'))
 source=json.loads((D/'source_rows.json').read_text());wall=np.array([r['end_to_end_wall_s'] for r in rr]);m.update(cross_axis=cross_metrics(ss,start,goal,m['scored_end_s']),wall_median_s=float(np.median(wall)),wall_p90_s=float(np.percentile(wall,90)),source_gaps=observation_gap_diagnostics(source,rr),requests=len(q),captures=len(ss),physics_samples=len(physics['simulation_time_s']),raw_contact_samples=len(cc),raw_contacts_zero=not any(c['contact'] for c in cc),sealed_items=len(seal['artifact_sha256']),videos=videos)
 return m,dict(D=D,ss=ss,rows=rr,requests=q,physics=physics,identity=identity,episode=episode)
