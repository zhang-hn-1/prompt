"""Offline physical/data assessment; only runs after parent post-close seal."""
from pathlib import Path
import json,hashlib,datetime,collections
import numpy as np
import cv2
from route_geometry_audit import project
H=Path(__file__).resolve().parent;O=H/'depth_local_curve_run';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 p=json.loads((H/'depth_local_curve_protocol.json').read_text());status=json.loads((H/'depth_local_curve_status.json').read_text());assert status['exit_code']==0 and p['sealed_utc']<status['start_utc']
 m=json.loads((O/'artifact_manifest_post_close.json').read_text())
 for f,s in m['artifact_sha256'].items():assert sha(f)==s,f
 for f,s in p['source_sha256'].items():assert sha(f)==s,f
 metrics=json.loads((O/'metrics.json').read_text());records=json.loads((O/'policy_records.json').read_text());clocks=json.loads((O/'camera_clocks.json').read_text());identity=json.loads((O/'identity.json').read_text());ph=np.load(O/'physics.npz');arrays=np.load(O/'episode_arrays.npz');contacts=json.loads((O/'physics_contacts.json').read_text());N=len(records)
 assert len(clocks)==N+1 and len(ph['simulation_time_s'])==5*N+1
 ref=np.load(H/'oracle_rgb_shadow_reference.npz')['pose'][:,:2];progress,lateral,_=project(arrays['positions_world_xy'],ref)
 occ=np.load(O/'offline_occupancy_label.npz');maprows=[];planning=[];windows=[]
 for i,r in enumerate(records):
  grid=np.load(O/'maps'/f'frame_{i:03d}.npz');labels=grid['labels'];codes=grid['provenance'];t=float(grid['time_s']);assert t==r['local']['time_s']
  known=np.argwhere(labels>=0);w=grid['origin']+known*float(grid['resolution']);index=np.rint((w-occ['origin'])/float(occ['resolution'])).astype(int);inside=((index>=0)&(index<np.array(occ['labels'].shape))).all(1)
  truth=np.full(len(known),-1);truth[inside]=occ['labels'][index[inside,0],index[inside,1]];kind=labels[known[:,0],known[:,1]]
  assert np.isfinite(grid['floor_time'][labels==0]).all() and np.isfinite(grid['hazard_time'][labels==1]).all()
  assert np.all(grid['floor_time'][labels==0]<=t+1e-6) and np.all(grid['hazard_time'][labels==1]<=t+1e-6)
  maprows.append(dict(frame=i,time_s=t,current_floor=int((codes==1).sum()),historical_floor=int((codes==2).sum()),current_hazard=int((codes==3).sum()),historical_hazard=int((codes==4).sum()),unknown=int((labels==-1).sum()),floor_occupied_overlap_proxy=int(((kind==0)&(truth==1)).sum()),history_hazard_occupied_overlap_proxy=int(((codes[known[:,0],known[:,1]]==4)&(truth==1)).sum())))
  if r['local']['local_plan_created']:
   planning.append(dict(frame=i,time_s=t,**r['local']['plan']))
   if r['local']['plan']['status']=='success':
    local=np.load(O/'local_plans'/f'frame_{i:03d}.npz')['pose'][:,:2];end=min(N,i+5);pr,er,_=project(arrays['positions_world_xy'][i:end+1],local)
    windows.append(dict(start_frame=i,end_frame=end,time_s=t,actual_window_progress_m=float(pr[-1]-pr[0]),entry_error_m=float(er[0]),exit_error_m=float(er[-1]),peak_error_m=float(er.max())))
 videos=[]
 for name in ['stereo_view','third_person']:
  cap=cv2.VideoCapture(str(O/(name+'.mp4')));n=0
  while cap.read()[0]:n+=1
  fps=cap.get(cv2.CAP_PROP_FPS);cap.release();assert n==N+1 and fps==10.;videos.append(dict(name=name,frames=n,fps=fps))
 moving=[r for r in records if abs(r['request']['policy_protocol_main_command_rad_s'])>1e-9]
 sources=collections.Counter(r['request']['request_source'] for r in records)
 active=[r for r in moving if r['tracking'].get('valid')]
 pct=lambda rows,key:float(np.mean([bool(r['tracking'].get(key,False)) for r in rows])) if rows else None
 result=dict(assessed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),post_close_manifest_utc=m['saved_utc'],navigation={k:v for k,v in metrics.items() if not isinstance(v,list)},reference=dict(independent_fixed_reference=True,final_progress_m=float(progress[-1]),maximum_progress_m=float(progress.max()),lateral_rms_m=float(np.sqrt(np.mean(lateral**2))),lateral_peak_m=float(lateral.max())),acquisition=dict(policy_calls=N,captures=N+1,physics_samples=len(ph['simulation_time_s']),contact_samples=len(contacts),actual_contact_samples=sum(bool(x['contact']) for x in contacts),max_clock_age_s=max(abs(x['raw_time_s']-v) for x in clocks for v in x['eyes'].values()),videos=videos),planning=dict(attempts=len(planning),status_counts=dict(collections.Counter(r['status'] for r in planning)),rows=planning,windows=windows),control=dict(request_sources=dict(sources),moving_blocks=len(moving),tracking_moving_blocks=len(active),protocol_clipped_fraction=pct(active,'protocol_clipped'),angle_target_at_bound_fraction=pct(active,'angle_target_at_bound'),v1_outside_support_fraction=pct(active,'requested_curvature_outside_v1_support'),curvature_absolute_p50=float(np.median([abs(r['tracking']['requested_curvature_1_m']) for r in active])) if active else None),map_rows=maprows,latency=dict(policy_p50_s=float(np.percentile([r['policy_wall_s'] for r in records],50)),policy_p95_s=float(np.percentile([r['policy_wall_s'] for r in records],95)),plan_p50_s=float(np.percentile([r.get('seconds',0.) for r in planning],50)) if planning else None,plan_p95_s=float(np.percentile([r.get('seconds',0.) for r in planning],95)) if planning else None,capture_partial_p50_s=float(np.percentile([r['capture_partial_wall_s'] for r in clocks],50)),capture_partial_p95_s=float(np.percentile([r['capture_partial_wall_s'] for r in clocks],95)),causal_online_not_wall_10hz=True),navigation_gate_pass=bool(metrics['success'] and not any(x['contact'] for x in contacts)),interface_gate_pending_full_replay=True,source_rgb_gate_unchanged_rejected=True,full_3d_certified=False,formal_matrix=False,whole_research_complete=False,scope='One independent privileged depth/knownpose partial-map local-curve package, explicit unknown risk and Oracle control domain; not single-factor AB/RGB/estimatedpose/dynamic physical certificate.')
 (H/'depth_local_curve_assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['map_rows','planning']},indent=2));print('PLANS',result['planning']['status_counts'])
if __name__=='__main__':main()
