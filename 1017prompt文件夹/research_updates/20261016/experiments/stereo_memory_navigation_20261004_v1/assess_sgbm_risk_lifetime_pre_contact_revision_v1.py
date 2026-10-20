"""Frozen measured paired outcomes; no cursor based success or inferred free space."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
from baseline35_metrics_v1 import assess,observation_gap_diagnostics
from repeated_exposure_memory_v1 import RepeatedExposureMemoryGrid
H=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def cross_metrics(states,start,goal,end):
 xy=np.array([s['position'][:2] for s in states if s['time_s']<=end+1e-5]);u=(goal-start)/np.linalg.norm(goal-start)
 e=(xy-start)@np.array([-u[1],u[0]])
 return dict(rms_m=float(np.sqrt(np.mean(e*e))),peak_m=float(np.max(np.abs(e))),scope='same frozen start-goal line, not dynamically changed local references')
def arm(name,design):
 D=H/('sgbm_risk_lifetime_e1_'+name+'_run_v1');seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
 assert seal['status']['status']=='finished' and seal['status']['child_returncode']==0
 for f,d in seal['artifact_sha256'].items():assert sha(Path(f))==d,f
 contract=json.loads((D/'ablation_contract.json').read_text());assert contract['design']==design and contract['arm']==name
 identity=json.loads((D/'identity.json').read_text());assert identity['known_GT_pose_control'] and identity['source_GT_pose_registration'] and identity['reset_count']==1 and not identity['internal_reset']
 expected='RepeatedExposureMemoryGrid' if name=='history' else 'CurrentExposureRiskGrid';assert expected in identity['actual_grid_class']
 ss=json.loads((D/'states.json').read_text());cc=json.loads((D/'physics_contacts.json').read_text());rr=json.loads((D/'policy_records.json').read_text());tt=json.loads((D/'task_episode.json').read_text())['actual_episode']
 start=np.array(tt['start_position_world_m'][:2]);goal=np.array(tt['goal_position_world_m'][:2]);m=assess(ss,cc,start,goal)
 requests=np.load(D/'requests.npz')['requests'];assert len(rr)==len(requests) and np.array_equal(requests,np.array([r['request'] for r in rr],np.float32))
 assert np.allclose(np.array([s['time_s'] for s in ss]),np.arange(len(ss))*.1,atol=2e-6,rtol=0)
 assert all(r['known_GT_pose'] and not r['native_pose_used'] for r in rr)
 assert not any(r.get('measured_free',0)>0 for r in rr)
 physics=np.load(D/'physics.npz');assert np.allclose(physics['simulation_time_s'],np.arange(len(physics['simulation_time_s']))*.02,atol=2e-6,rtol=0)
 videos={}
 for p in D.glob('*.mp4'):
  v=cv2.VideoCapture(str(p));fps=v.get(cv2.CAP_PROP_FPS);n=0
  while True:
   ok,_=v.read()
   if not ok:break
   n+=1
  v.release();assert n==len(ss) and fps==10.;videos[p.name]=n
 assert videos
 clocks=json.loads((D/'camera_clocks.json').read_text());assert len(clocks)==len(ss)
 for c in clocks:assert abs(c['eyes']['left']-c['raw_time_s'])<1e-7 and abs(c['eyes']['right']-c['raw_time_s'])<1e-7
 source=json.loads((D/'source_rows.json').read_text()) if (D/'source_rows.json').exists() else []
 wall=np.array([r['end_to_end_wall_s'] for r in rr]);result=dict(metrics=m,cross_track=cross_metrics(ss,start,goal,m['scored_end_s']),wall_median_s=float(np.median(wall)),wall_p90_s=float(np.percentile(wall,90)),videos_decoded=videos,manifest_items_verified=len(seal['artifact_sha256']),all_contacts_available=all(c['available'] for c in cc),raw_contact_hits=sum(c['collision'] for c in cc if c['available']),source_gaps=observation_gap_diagnostics(source,rr),rows=len(rr),risk_lifetime_labels=sorted(set(r['hazard_lifetime'] for r in rr)),seal_sha256=sha(D/'artifact_manifest_post_close.json'))
 return result,D,ss,rr,requests,physics,identity,tt

def main():
 design=json.loads((H/'sgbm_risk_lifetime_design_v1.json').read_text())
 for group in ['source_sha256','input_sha256']:
  for f,d in design[group].items():assert sha(Path(f))==d,f
 a,AD,ass,ar,aq,ap,ai,at=arm('history',design);b,BD,bss,br,bq,bp,bi,bt=arm('current',design)
 assert at==bt
 for k in ['camera_rig','pid_config','inner_config','policy_MRO','tracker_MRO','frozen_source_method','source_sha256','input_sha256']:assert ai[k]==bi[k],k
 n=min(len(aq),len(bq));different=np.flatnonzero(np.any(aq[:n]!=bq[:n],axis=1));first=int(different[0]) if len(different) else None
 before=first*.1 if first is not None else min(ass[-1]['time_s'],bss[-1]['time_s'])
 fields=[k for k in ap.files if not k.startswith('raw_')]
 am=ap['simulation_time_s']<=before+1e-6;bm=bp['simulation_time_s']<=before+1e-6
 prefix=all(np.array_equal(ap[k][am],bp[k][bm]) for k in fields) and np.array_equal(aq[:first],bq[:first])
 # Distinct images before first effect are reported, never assumed identical.
 fa={p.name:sha(p) for p in (AD/'source_inputs').glob('*.npz') if int(p.stem.split('_')[-1])<=int(round(before*10))}
 fb={p.name:sha(p) for p in (BD/'source_inputs').glob('*.npz') if int(p.stem.split('_')[-1])<=int(round(before*10))}
 image_identical=fa==fb
 geometry_identical=True
 for f in range((first+1) if first is not None else n):
  ra=json.loads((AD/'raw_point_memory_snapshots'/f'frame_{f:03d}.json').read_text())['records'];rb=json.loads((BD/'raw_point_memory_snapshots'/f'frame_{f:03d}.json').read_text())['records']
  if len(ra)!=len(rb) or any(x['frame']!=y['frame'] or x['time_s']!=y['time_s'] or not np.array_equal(np.asarray(x['world']),np.asarray(y['world'])) for x,y in zip(ra,rb)):geometry_identical=False

 # Causal current-arm record replay only to check removal of positive-history risk.
 hist=RepeatedExposureMemoryGrid(np.array(bt['start_position_world_m'][:2])-20,10,static_scene_assumption=True);removed=0
 for r in br:
  f=r['frame'];snap=json.loads((BD/'raw_point_memory_snapshots'/f'frame_{f:03d}.json').read_text());records=snap['records']
  envelope=dict(frame=r['source_frame'],time_s=r['source_time_s'],records=records)
  root=np.array(bss[f]['position']);hist.set_source(envelope);hist.update_points([],root,r['time_s'])
  with np.load(BD/'maps'/f'frame_{f:03d}.npz') as z:
   assert not np.any(z['labels']==0)
   removed+=int(np.sum((hist.labels==1)&(z['labels']==-1)))
 no_worse=a['raw_contact_hits']<=b['raw_contact_hits'] and a['all_contacts_available'] and b['all_contacts_available']
 benefit=bool(first is not None and prefix and image_identical and geometry_identical and removed>0 and no_worse and ((a['metrics']['success'] and not b['metrics']['success']) or (a['metrics']['success'] and b['metrics']['success'] and a['metrics']['arrival_time_s']<b['metrics']['arrival_time_s'] and a['cross_track']['rms_m']<=b['cross_track']['rms_m']+.1 and a['cross_track']['peak_m']<=b['cross_track']['peak_m']+.2)))
 result=dict(history=a,current=b,first_actual_request_difference_frame=first,physical_and_request_prefix_exact=prefix,prefix_source_input_files_byte_identical=image_identical,prefix_raw_point_geometry_exact=geometry_identical,history_positive_current_unknown_cell_sample_count=removed,memory_navigation_benefit=benefit,not_full3d_safety=True,not_strict_coldstart=True,no_new_model=True,no_native_pose_admission=True,whole_mission_complete=False,scope='If exact causal prerequisites fail, paired observational runs only; limited progress differences never navigation success',assessor_sha256=sha(Path(__file__)))
 p=H/'sgbm_risk_lifetime_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()
