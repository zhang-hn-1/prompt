"""One bounded unchanged ORB consumer in LEFT-only visible domain; development only."""
from pathlib import Path
import json,numpy as np,hashlib,datetime
from feature_tracks import extract
from active_observation import guarded_predict
from timed_static_base_self_mask import TimedStaticBaseSelfMask
from dynamic_self_mask import joints_from_proprio
H=Path(__file__).resolve().parent;D=H/'minimum_prefix_transition_navigation_run_v1';O=H/'left_temporal_missing_divider_development_v1';O.mkdir(exist_ok=False)
frames=[215,220,225,325,330,335];K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),frames=frames,maximum_queries=6,source_history='last four past left exposures <=2s; no future; no depth/target in predictor',only_change='query eye right->left on independently labelled latest development sequence, same ORB/matching/triangulation/sigma thresholds',source_gate='95% within10cm / p90<=10cm / N>=100 / >=2epochs target>=100 unchanged',known_GT_camera_pose_diagnostic=True,old_right_temporal_and_global_depth_REJECT_preserved=True,no_new_network_or_fit=True,no_actor_install=True,not_fresh_heldout=True,not_navigation=True)
(O/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');cad=TimedStaticBaseSelfMask();cache={};predictions=[]
def feature(i):
 if i not in cache:
  rgb=np.load(D/'source_inputs'/f'frame_{i:03d}.npz');pose=np.load(D/'known_poses'/f'frame_{i:03d}.npz');hist=np.load(D/'history'/f'frame_{i:03d}.npz')['proprio_history'].copy();hist[:,:9]=0;mask=cad.project(joints_from_proprio(hist))['left'];xy,desc,size=extract(rgb['left_rgb'],mask)
  cache[i]=dict(frame=i,time_s=float(pose['time_s']),eye='left',xy=xy,desc=desc,size=size,pose=pose['left_camera_usd_world'])
 return cache[i]
for i in frames:
 q=feature(i);sources=[feature(j) for j in range(i-20,i,5)];a=guarded_predict(q,sources,K);b=guarded_predict(q,sources+[dict(time_s=q['time_s']+.1,FORBIDDEN_FUTURE=True)],K);assert a==b;predictions.append(dict(frame=i,prediction=a,future_payload_ignored=True))
(O/'predictions.json').write_text(json.dumps(predictions,indent=2)+'\n');seal=hashlib.sha256((O/'predictions.json').read_bytes()).hexdigest();(O/'prediction_seal.json').write_text(json.dumps(dict(sha256=seal,labels_not_loaded_until_this_seal=True),indent=2)+'\n')
# Only now introduce sealed GT labels; target identity is solely an evaluator.
obj=next(x for x in json.loads((H/'minimum_prefix_transition_sparse_membership_v2/scene_collision_shapes.json').read_text()) if x['path']=='/E1Detour/Divider/divider_06');centre=np.array(obj['position']);E=[];rows=[]
for p in predictions:
 i=p['frame'];q=feature(i);gt=np.load(D/'frames'/f'frame_{i:03d}.npz')['left_depth_m'];T=q['pose'];R=T[:3,:3]@np.diag([1,-1,-1]);e=[];nt=0
 for point in p['prediction']['points']:
  j=point['query'][2];u,v=q['xy'][j];x,y=np.rint([u,v]).astype(int);z=gt[y,x];truth=T[:3,3]+R@np.array([(u-319.5)/K[0,0]*z,(v-239.5)/K[1,1]*z,z]);error=float(np.linalg.norm(np.array(point['world'])-truth)) if np.isfinite(z) and z>0 else float('inf');e.append(error);nt+=int(np.isfinite(truth).all() and np.linalg.norm(truth[:2]-centre[:2])<=obj['radius']+.001 and abs(truth[2]-centre[2])<=obj['height']/2+.001)
 E.extend(e);rows.append(dict(frame=i,total_points=len(e),target_points=nt,within10cm=sum(v<=.1 for v in e),query_features=len(q['xy'])))
e=np.array(E);gates=dict(point100=len(e)>=100,geometry95=len(e)>0 and float(np.mean(e<=.1))>=.95,p90=len(e)>0 and float(np.percentile(e,90))<=.1,target_two100epochs=sum(r['target_points']>=100 for r in rows)>=2)
r=dict(protocol=protocol,decision='PASS_DEVELOPMENT_ONLY_REQUIRE_FRESH_WHOLE_SEQUENCE' if all(gates.values()) else 'REJECT_LEFT_TEMPORAL_MISSING_DIVIDER_DEVELOPMENT',points=len(e),within10cm_fraction=None if not len(e) else float(np.mean(e<=.1)),unconditional_p90_m=None if not len(e) else float(np.percentile(e,90)),target_points=sum(r['target_points'] for r in rows),gates=gates,rows=rows,source_truth_loaded_after_prediction_seal=True,no_actor_install=True,no_free_volume=True,no_navigation=True,whole_research_complete=False)
(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
