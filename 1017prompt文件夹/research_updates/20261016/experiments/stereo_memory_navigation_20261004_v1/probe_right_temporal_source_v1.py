"""One bounded old ORB current-right consumer diagnostic; never a new network."""
from pathlib import Path
import json,numpy as np,hashlib,datetime
from feature_tracks import extract
from active_observation import guarded_predict
H=Path(__file__).resolve().parent;D=H/'frontal_obstacle_acquisition_run_v1';O=H/'right_temporal_source_development_v1';O.mkdir(exist_ok=False);K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
frames=[60,70,80,90,100,110,120,130,140,160,180,200];static=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');cache={};predictions=[]
# Freeze before any new labels; minimum target100 kept from acquisition protocol.
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),maximum_query_exposures=12,frames=frames,only_source_consumer_change='current right query plus last four past right exposures, existing frozen ORB/extraction/triangulation unchanged',target_gate='two epochs>=100 actual target points unchanged; source95%10cm/p90<=10cm and100points original',old_ORB_source_REJECT_preserved=True,not_new_network=True,no_fitting=True,not_navigation=True,not_fresh_heldout=True)
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
def feature(i):
 if i not in cache:
  rgb=np.load(D/'source_inputs'/f'frame_{i:03d}.npz');pose=np.load(D/'known_poses'/f'frame_{i:03d}.npz');xy,desc,size=extract(rgb['right_rgb'],static['right']);cache[i]=dict(frame=i,time_s=float(pose['time_s']),eye='right',xy=xy,desc=desc,size=size,pose=pose['right_camera_usd_world'])
 return cache[i]
for i in frames:
 q=feature(i);sources=[feature(j) for j in range(max(0,i-20),i,5)];a=guarded_predict(q,sources,K);b=guarded_predict(q,sources+[dict(time_s=q['time_s']+.1,FORBIDDEN_FUTURE=True)],K);assert a==b;predictions.append(dict(frame=i,prediction=a,future_payload_ignored=True))
(O/'predictions.json').write_text(json.dumps(predictions,indent=2)+'\n');seal=hashlib.sha256((O/'predictions.json').read_bytes()).hexdigest();(O/'prediction_seal.json').write_text(json.dumps(dict(sha256=seal,labels_not_loaded_until_this_seal=True),indent=2)+'\n')
# Labels now permitted; source received no depth, target or geometry.
errors=[];rows=[];target=np.array([0.,-1.2,.5])
for p in predictions:
 i=p['frame'];q=feature(i);gt=np.load(D/'frames'/f'frame_{i:03d}.npz')['right_depth_m'];T=q['pose'];R=T[:3,:3]@np.diag([1,-1,-1]);e=[];nt=0
 for point in p['prediction']['points']:
  j=point['query'][2];u,v=q['xy'][j];x,y=np.rint([u,v]).astype(int);z=gt[y,x];truth=T[:3,3]+R@np.array([(u-319.5)/K[0,0]*z,(v-239.5)/K[1,1]*z,z]);err=float(np.linalg.norm(np.array(point['world'])-truth)) if np.isfinite(z) and z>0 else float('inf');e.append(err);nt+=int(np.isfinite(truth).all() and np.linalg.norm(truth[:2]-target[:2])<=.351 and abs(truth[2]-.5)<=.501)
 errors.extend(e);rows.append(dict(frame=i,total_points=len(e),target_points=nt,within10cm=sum(x<=.1 for x in e),query_features=len(q['xy'])))
e=np.array(errors);gates=dict(point100=len(e)>=100,two100epochs=sum(r['total_points']>=100 for r in rows)>=2,geometry95=len(e)>0 and float(np.mean(e<=.1))>=.95,p90=len(e)>0 and float(np.percentile(e,90))<=.1,target_two100epochs=sum(r['target_points']>=100 for r in rows)>=2)
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),points=len(e),within10cm_fraction=None if not len(e) else float(np.mean(e<=.1)),unconditional_p90_m=None if not len(e) else float(np.percentile(e,90)),target_points=sum(r['target_points'] for r in rows),gates=gates,decision='PASS_ONLY_DEVELOPMENT_REQUIRE_FRESH_WHOLE_SEQUENCE' if all(gates.values()) else 'REJECT_RIGHT_TEMPORAL_DEVELOPMENT_NO_NEW_PHYSICS',old_ORB_REJECT_preserved=True,no_actor_install=True,no_free_volume=True,no_navigation=True,whole_research_complete=False);(O/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');(O/'rows.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(result,indent=2))
