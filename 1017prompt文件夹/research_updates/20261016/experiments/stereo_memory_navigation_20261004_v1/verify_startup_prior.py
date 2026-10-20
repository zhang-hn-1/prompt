from pathlib import Path
import json,datetime,hashlib,numpy as np
from perfect_visible_points import perfect_packet
from dense_source_points import records
from point_support_shadow import PointSupportMap,replay_prefix
from startup_prior import ImmutableStartupPrior
H=Path(__file__).resolve().parent;out=H/'startup_prior_runs';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();p=json.loads((H/'startup_prior_protocol.json').read_text());m=json.loads((out/'manifest.json').read_text());a=json.loads((out/'assessment.json').read_text());checks=0
for kind in ['source_sha256','input_sha256']:
 for f,d in p[kind].items():assert sha(f)==d,f;checks+=1
for r in m['rows']:
 for f,d in [('path','sha256'),('packet_path','packet_sha256'),('map_path','map_sha256')]:assert sha(r[f])==r[d];checks+=1
assert p['utc']<m['saved_utc']<a['utc'];assert len(m['rows'])==65 and m['future_trajectory_not_read'];mask=np.load(p['mask']);history=PointSupportMap(p['origin'],p['shape'],.1);packets=[];pr=np.load(out/'immutable_prior.npz');prior=ImmutableStartupPrior(pr['cells'],float(pr['time_s']),source='task_assumption',trusted=True)
for r in m['rows']:
 frame=r['frame'];raw=np.load(H/'dense_adapter_heldout_run/frames'/f'frame_{frame:03d}.npz');curr={k:raw[k] for k in ['left_depth_m','right_depth_m','left_camera_usd_world','right_camera_usd_world','root_position_world_m','time_s']};fresh=perfect_packet(curr,mask['left'],mask['right'],frame);saved=np.load(r['packet_path'])
 for k in fresh:assert np.array_equal(fresh[k],saved[k]),(frame,k)
 pts=records(fresh);t=fresh['time_s'];history.update(pts,t,0.);snapshot=history.snapshot(t);maps=np.load(r['map_path']);rec=json.loads(Path(r['path']).read_text())
 for k in ['labels','provenance','floor_time','hazard_time']:assert np.array_equal(snapshot[k],maps[k]),(frame,k)
 if frame>=60:assert prior.evaluate(maps['labels'],maps['current_disk'],t)==rec['with_prior']
 assert rec['action_authorized'] is False;packets.append(dict(time_s=t,points=pts,pose_sigma_m=0.))
class Trap(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('future payload read')
  return super().__getitem__(k)
causal=0
for frame in (0,60,160):
 t=m['rows'][frame//5]['time_s'];prefix=[q for q in packets if q['time_s']<=t];future=[Trap(time_s=q['time_s']) for q in packets if q['time_s']>t];base=replay_prefix(prefix,t,p['origin'],p['shape'],.1).snapshot(t);changed=replay_prefix(prefix+future,t,p['origin'],p['shape'],.1).snapshot(t);saved=np.load(out/f'map_{frame:03d}.npz')
 for k in ['labels','provenance','floor_time','hazard_time']:assert np.array_equal(base[k],changed[k]) and np.array_equal(base[k],saved[k])
 causal+=1
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),passed=True,identity_checks=checks,exact_source_packet_reconstructions=65,exact_history_maps_rebuilt=65,actual_time_future_payload_truncation_checks=causal,prior_nonobservation=True,scientific_conditions_rejected=True,whole_research_complete=False)
(H/'startup_prior_author_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
