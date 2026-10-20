"""Checks one completed fixed-command sensor probe; no navigation success claim."""
from pathlib import Path
import hashlib,json,datetime
import numpy as np,cv2
from retreat_observation_schedule import request
from masked_source_pair import paired_sources
from dynamic_self_mask import RobotSelfMask
from stereo_geometry import restore_native
H=Path(__file__).resolve().parent;o=H/'retreat_observation_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();status=json.loads((H/'retreat_observation_status.json').read_text());assert status['status']=='finished' and status['exit_code']==0
protocol=json.loads((H/'retreat_observation_protocol.json').read_text());checks=0
for m in [protocol['source_sha256'],json.loads((o/'artifact_manifest_post_close.json').read_text())['artifact_sha256']]:
 for f,s in m.items():assert sha(f)==s,f;checks+=1
rows=json.loads((o/'policy_records.json').read_text());states=json.loads((o/'states.json').read_text());clocks=json.loads((o/'camera_clocks.json').read_text());ph=np.load(o/'physics.npz');assert len(rows)==180 and len(states)==181 and len(ph['simulation_time_s'])==901
nominal_differences=0
for i,r in enumerate(rows):
 a=request(r['local']['time_s']);np.testing.assert_equal(r['local']['request'],a);np.testing.assert_equal(np.load(o/'policy_inputs'/f'frame_{i:03d}.npz')['request'],a);np.testing.assert_equal(ph['raw_command_target'][5*i+1:5*i+6],a.reshape(5,2));assert r['local']['nominal_policy_fields_not_applied'];nominal_differences+=int(np.any(a!=r['local']['nominal_request']))
cad=RobotSelfMask();packet_fields=0;correct=0;finite=0;total=0;source_rows=[]
for f in sorted((o/'source_predictions').glob('*.npz')):
 d=np.load(f);i=int(d['frame']);t=float(d['time_s']);pose=np.load(o/'known_poses'/f'frame_{i:03d}.npz');hist=np.load(o/'history'/f'frame_{i:03d}.npz');known=dict(left_camera_usd_world=pose['left_camera_usd_world'],right_camera_usd_world=pose['right_camera_usd_world'],root_position_world_m=states[i]['position'],time_s=float(pose['time_s']));raw=dict(depth_m=d['depth_m'],confidence=d['confidence'],disparity_native_px=restore_native(d['disparity_px']),frame=i,time_s=t);old,new,masks=paired_sources(raw,known,hist['proprio_history'],cad,t)
 saved=np.load(o/'sensor_packets'/f'source_{i:03d}.npz')
 for k,v in new.items():np.testing.assert_equal(v,saved[k]);packet_fields+=1
 mask=np.load(o/'masks'/f'frame_{i:03d}.npz')
 for eye in ['left','right']:np.testing.assert_equal(masks[eye],mask[eye]);packet_fields+=1
 truth=np.load(o/'offline_truth'/f'frame_{i:03d}.npz');q=new['query_pixel_id'].astype(int);yy=q//640;xx=q%640;z=truth['left_depth_m'][yy,xx];T=truth['left_camera_usd_world'];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(q))]
 with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(new['world']-actual,axis=1)
 n=int(np.isfinite(err).sum());good=int((np.isfinite(err)&(err<=.1)).sum());total+=len(q);finite+=n;correct+=good;source_rows.append(dict(frame=i,time_s=t,queries=len(q),finite_truth=n,correct_10cm=good,root_position=states[i]['position']))
contacts=json.loads((o/'physics_contacts.json').read_text());videos=[]
for name in ['stereo_view','third_person']:
 cap=cv2.VideoCapture(str(o/(name+'.mp4')));n=0
 while True:
  ok,_=cap.read()
  if not ok:break
  n+=1
 fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==181 and fps==10;videos.append(dict(name=name,frames=n,fps=fps))
identity=json.loads((o/'identity.json').read_text());assert identity['execution_role']=='fixed_retreat_observation_probe' and identity['nominal_policy_outputs_not_applied'] and identity['local_goal_m']==2.6
pos=np.array([s['position'] for s in states]);speed=np.linalg.norm(np.array([s['velocity'][:2] for s in states]),axis=1);stopped=0;qual=[]
for i in range(120,181):
 stopped=stopped+1 if speed[i]<.05 else 0
 if stopped>=5:qual.append(i)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),identity_checks=checks,request_blocks_exact=180,physics=901,captures=181,inferences=len(source_rows),packet_mask_fields_exact=packet_fields,nominal_policy_request_differences=nominal_differences,nominal_policy_not_control=True,contact_samples=sum(bool(x['contact']) for x in contacts),positions_0_6_12_18=pos[[0,60,120,180]].tolist(),actual_retreat_x_m=float(pos[60,0]-pos[120,0]),first_5_consecutive_XY_low_speed_after12s=None if not qual else states[qual[0]]['time_s'],all_source_queries=total,finite_truth=finite,correct_10cm=correct,correct_10cm_unconditional_fraction=correct/total if total else 0.,source_rows=source_rows,videos=videos,max_exposure_delta_s=max(abs(c['raw_time_s']-e) for c in clocks for e in c['eyes'].values()),whole_research_complete=False,formal_navigation_not_run=True,original_source_gate_not_rescored=True,observability_and_model_accuracy_separate=True)
(H/'retreat_observation_assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='source_rows'},indent=2))
