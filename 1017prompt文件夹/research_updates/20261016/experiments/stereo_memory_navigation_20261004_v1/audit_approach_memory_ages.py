"""Retrospective age classification; truth-filtered aggregates NEVER feed map."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent;cases=['fresh_rolling_prior_shadow_current_run','occluded_obstacle_memory_navigation_history_run'];frames=[60,100,140,180,220,260,295];inputs=[Path(__file__).resolve(),H/'approach_visibility_diagnostic.json',H/'point_support_shadow.py'];files={}
for case in cases:
 o=H/case;files[case]=sorted([p for p in (o/'sensor_packets').glob('*.npz') if int(p.stem.split('_')[-1])<=295]);inputs.append(o/'states.json')
 for p in files[case]:inputs.extend([p,o/'offline_truth'/f"frame_{int(p.stem.split('_')[-1]):03d}.npz"])
 for i in frames:inputs.append(o/'maps'/f'frame_{i:03d}.npz')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),files={str(p):sha(p) for p in inputs},maximum_packet_frame=295,selected_frames=frames,definition='query truth endpoint on frozen divider-cylinder proxy and predicted endpoint within .10m; hazard z (.05,1.5]; cell age uses unchanged rounding',not_a_candidate_truth_filter=True,old_ttl_s=10,online_behavior_unchanged=True)
(H/'approach_memory_ages_preanalysis.json').write_text(json.dumps(protocol,indent=2)+'\n')
geom=json.loads((H/'approach_visibility_diagnostic.json').read_text());centres=np.array(geom['divider_centres']);radius=geom['radius_m'];results=[]
for case in cases:
 o=H/case;states=json.loads((o/'states.json').read_text());cache=[]
 for p in files[case]:
  s=np.load(p);i=int(s['frame']);truth=np.load(o/'offline_truth'/f'frame_{i:03d}.npz');ids=s['query_pixel_id'].astype(int);yy=ids//640;xx=ids%640;z=truth['left_depth_m'][yy,xx];rays=np.c_[(xx-319.5)/584.8888888888889,(yy-239.5)/584.8888888888889,np.ones(len(ids))];T=truth['left_camera_usd_world']
  with np.errstate(invalid='ignore'):actual=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(s['world']-actual,axis=1)
  proxy=np.zeros(len(ids),bool)
  for c in centres:proxy|=(np.linalg.norm(actual[:,:2]-c[:2],axis=1)<=radius+.005)&(actual[:,2]>=-.005)&(actual[:,2]<=1.005)
  ok=proxy&np.isfinite(err)&(err<=.1)&(s['world'][:,2]>.05)&(s['world'][:,2]<=1.5)
  cache.append(dict(frame=i,time=float(s['time_s']),world=s['world'][ok],good_count=int(ok.sum())))
 for i in frames:
  m=np.load(o/'maps'/f'frame_{i:03d}.npz');t=float(m['time_s']);latest=np.full(m['labels'].shape,-np.inf);n=0
  for s in cache:
   if s['time']>t:continue
   xy=s['world'][:,:2];cell=np.rint((xy-m['origin'])/float(m['resolution'])).astype(int);inside=(cell>=0).all(1)&(cell<m['labels'].shape).all(1);cell=cell[inside];np.maximum.at(latest,(cell[:,0],cell[:,1]),s['time']);n+=s['good_count']
  finite=np.isfinite(latest);ages=t-latest[finite];active=finite&(t-latest<=10);expired=finite&~active;observed=m['labels']==1
  results.append(dict(case=case,frame=i,time_s=t,correct_divider_queries_past_prefix=n,all_past_correct_divider_hazard_cells=int(finite.sum()),correct_cells_original_10s_alive=int(active.sum()),correct_cells_older_than_10s=int(expired.sum()),alive_correct_cells_current_primary_hazard=int((active&observed).sum()),expired_correct_cells_still_primary_hazard=int((expired&observed).sum()),primary_hazard_cells=int(observed.sum()),age_p10_p50_p90=np.percentile(ages,[10,50,90]).tolist() if len(ages) else [],actual_goal_progress_m=float(states[i]['position'][0]-states[0]['position'][0])))
result=dict(protocol=protocol,rows=results,truth_classification_posthoc_only=True,unbounded_hypothetical_retention_not_executed=True,no_controller_candidate_tested=True,whole_research_complete=False)
(H/'approach_memory_ages_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n')
for r in results:print(r)
