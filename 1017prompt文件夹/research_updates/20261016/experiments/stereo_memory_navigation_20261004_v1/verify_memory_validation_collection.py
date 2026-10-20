"""Closed physical/source evidence checks, after held prediction freeze."""
from pathlib import Path
import hashlib,json,datetime
import numpy as np,cv2
from memory_validation_schedule import request
from masked_source_pair import paired_sources
from dynamic_self_mask import RobotSelfMask
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();assert (H/'learned_memory_held_predictions/all_predictions_pretruth_sha256.json').exists();status=json.loads((H/'memory_validation_collection_status.json').read_text());assert status['status']=='finished_both_validation_sequences_sealed';protocol=json.loads((H/'memory_validation_collection_protocol.json').read_text());checks=0
for f,s in protocol['source_sha256'].items():assert sha(f)==s,f;checks+=1
results=[];cad=RobotSelfMask()
for kind in ['far','near']:
 o=H/f'memory_validation_{kind}_run';count=0
 for f,s in json.loads((o/'artifact_manifest_post_close.json').read_text())['artifact_sha256'].items():assert sha(f)==s,f;count+=1
 rows=json.loads((o/'policy_records.json').read_text());states=json.loads((o/'states.json').read_text());ph=np.load(o/'physics.npz');assert len(rows)==180 and len(states)==181 and len(ph['simulation_time_s'])==901
 for i,r in enumerate(rows):
  a=request(r['local']['time_s'],kind);np.testing.assert_equal(r['local']['request'],a);np.testing.assert_equal(np.load(o/'policy_inputs'/f'frame_{i:03d}.npz')['request'],a);np.testing.assert_equal(ph['raw_command_target'][5*i+1:5*i+6],a.reshape(5,2));assert r['local']['nominal_policy_fields_not_applied']
 fields=0;correct=0;total=0;finite=0
 for f in sorted((o/'source_predictions').glob('*.npz')):
  d=np.load(f);i=int(d['frame']);t=float(d['time_s']);pose=np.load(o/'known_poses'/f'frame_{i:03d}.npz');hist=np.load(o/'history'/f'frame_{i:03d}.npz');known=dict(left_camera_usd_world=pose['left_camera_usd_world'],right_camera_usd_world=pose['right_camera_usd_world'],root_position_world_m=states[i]['position'],time_s=float(pose['time_s']));raw=dict(depth_m=d['depth_m'],confidence=d['confidence'],disparity_native_px=restore_native(d['disparity_px']),frame=i,time_s=t);_,new,masks=paired_sources(raw,known,hist['proprio_history'],cad,t);saved=np.load(o/'sensor_packets'/f'source_{i:03d}.npz')
  for k,v in new.items():np.testing.assert_equal(v,saved[k]);fields+=1
  mask=np.load(o/'masks'/f'frame_{i:03d}.npz')
  for eye in ['left','right']:np.testing.assert_equal(masks[eye],mask[eye]);fields+=1
  truth=np.load(o/'offline_truth'/f'frame_{i:03d}.npz');q=new['query_pixel_id'].astype(int);yy=q//640;xx=q%640;z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))]
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(new['world']-actual,axis=1)
  finite+=int(np.isfinite(err).sum());correct+=int((np.isfinite(err)&(err<=.1)).sum());total+=len(q)
 contacts=json.loads((o/'physics_contacts.json').read_text());videos=[]
 for name in ['stereo_view','third_person']:
  cap=cv2.VideoCapture(str(o/(name+'.mp4')));n=0
  while True:
   ok,_=cap.read()
   if not ok:break
   n+=1
  fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==181 and fps==10;videos.append(dict(name=name,frames=n,fps=fps))
 results.append(dict(kind=kind,artifact_sha_checks=count,exact_requests=180,physics_samples=901,captures=181,sources=37,source_mask_fields_exact=fields,contact_samples=sum(bool(x['contact']) for x in contacts),queries=total,finite_truth=finite,correct_10cm=correct,videos=videos,not_navigation=True))
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_SHA_checks=checks,sequences=results,held_predictions_sealed_before_truth_scoring=True,whole_research_complete=False);(H/'memory_validation_collection_author_checks.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
