from pathlib import Path
import json,hashlib,datetime,numpy as np,cv2
from scipy.spatial import cKDTree
from bank_rock_observation_schedule import request
H=Path(__file__).resolve().parent;D=H/'bank_rock_observation_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();st=json.loads((H/'bank_rock_observation_stage_status_v1.json').read_text());assert st['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION' and st['complete'];seal=D/'artifact_manifest_post_close.json';assert sha(seal)==st['post_close_manifest_sha256'];man=json.loads(seal.read_text());assert man['complete'] and all(sha(f)==s for f,s in man['artifact_sha256'].items())
rows=json.loads((D/'policy_records.json').read_text());states=json.loads((D/'states.json').read_text());clocks=json.loads((D/'camera_clocks.json').read_text());contacts=json.loads((D/'contacts.json').read_text());ph=np.load(D/'physics.npz');task=json.loads((D/'task_episode.json').read_text());pose=json.loads((H/'bank_rock_observation_sensor_verification_v1.json').read_text())
for i,r in enumerate(rows):np.testing.assert_equal(r['request'],request(i));assert not r['planner_act_called']
for i,r in enumerate(rows):np.testing.assert_equal(ph['raw_command_target'][i*5+1:i*5+6],request(i).reshape(5,2))
assert not ph['raw_command_target'][:,0].any()
xy=np.asarray([s['position'][:2] for s in states]);v=np.asarray([s['velocity'][:2] for s in states]);lowcounts=[];n=0
for velocity in v:
 n=n+1 if np.linalg.norm(velocity)<.05 else 0;lowcounts.append(n)
truth=np.load(D/'offline_occupancy_label.npz');gtxy=truth['origin']+np.argwhere(truth['labels']>0)*float(truth['resolution']);Y,X=np.mgrid[:480,:640];ray=np.stack([(X-319.5)/584.8888888888889,(Y-239.5)/584.8888888888889,np.ones_like(X)],-1)
endchecks=[];intermediate=[]
for i in range(0,141,5):
 q=dict(np.load(D/'possible_sets'/f'frame_{i:03d}.npz'));points=q['possible_xy'];coverage=float((cKDTree(points).query(gtxy)[0]<=np.sqrt(2)*.05+1e-9).mean()) if len(points) else 0.;intermediate.append(dict(frame=i,time_s=float(q['time_s']),coverage=coverage,model_area_m2=len(points)*.01))
 if i not in [125,130,135,140]:continue
 views=[]
 for f in sorted((D/'partial_class_masks').glob(f'birth_{i:03d}_*.npz')):
  eye,vf=f.stem.split('_')[-2:];vf=int(vf);assert vf<=i;cl=np.load(f)['potential_hazard_mask'];gt=np.load(D/'offline_truth'/f'frame_{vf:03d}.npz');z=gt[eye+'_depth_m'];T=gt[eye+'_camera_usd_world'];mask=np.load(D/'masks'/f'frame_{vf:03d}.npz')[eye]
  with np.errstate(invalid='ignore'):world=T[:3,3]+(ray*z[:,:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
  valid=np.isfinite(z)&np.isfinite(world).all(-1)&(z>=.5)&(z<=3)&(world[:,:,2]>.05)&(world[:,:,2]<=1.5)&(np.linalg.norm(world-gt['root_position_world_m'],axis=-1)>.55)&~mask
  precision=float((cl&valid).sum()/cl.sum()) if cl.any() else 0.;views.append(dict(frame=vf,eye=eye,precision=precision,pixels=int(cl.sum())))
 endchecks.append(dict(frame=i,coverage=coverage,area_m2=len(points)*.01,views=views,passed=coverage>=.95 and len(points)*.01<=6 and bool(views) and all(w['precision']>=.95 for w in views)))
videos={}
for name in ['stereo_view','third_person']:
 c=cv2.VideoCapture(str(D/(name+'.mp4')));count=0
 while True:
  ok,im=c.read()
  if not ok:break
  count+=1
 c.release();videos[name]=count
metrics=json.loads((D/'metrics.json').read_text());checks=dict(complete=len(rows)==140 and len(clocks)==141 and len(states)==141 and abs(metrics['elapsed_s']-14.)<1e-5,no_contact=not any(c['contact'] or c['collision'] for c in contacts),safe_start_true_displacement=float(np.linalg.norm(xy-xy[0],axis=1).max())<=.7,end_five_world_XY_low=lowcounts[-1]>=5,pose_gate=pose['pose_gate'],all_four_frozen_final_source_sets=all(c['passed'] for c in endchecks) and len(endchecks)==4,full_videos=all(n==141 for n in videos.values()),same_eye_exposure=max(abs(c['eyes']['left']-c['eyes']['right']) for c in clocks)==0.)
r=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='PASS_LOCAL_ROCKING_OBSERVATION_ONLY' if all(checks.values()) else 'REJECT_UNIQUE_ROCKING_OBSERVATION',checks=checks,actual_requests_exact=len(rows),dispatch_ticks_exact=len(rows)*5,max_true_XY_displacement_m=float(np.linalg.norm(xy-xy[0],axis=1).max()),end_low_count=lowcounts[-1],final_source=endchecks,all_intermediate_diagnostic=intermediate,intermediate_best_not_acceptance=True,videos=videos,source_only=True,navigation_executed=False,model_not_observed_free=True,ideal_bodygyro_and_initial_gauge_privileged=True,old_source_and_D4_reject_preserved=True,no_future_GT_actor=True,whole_research_complete=False,original35_formal_not_triggered=True,code_sha256=sha(__file__))
p=H/'bank_rock_observation_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ['final_source','all_intermediate_diagnostic']},indent=2))
