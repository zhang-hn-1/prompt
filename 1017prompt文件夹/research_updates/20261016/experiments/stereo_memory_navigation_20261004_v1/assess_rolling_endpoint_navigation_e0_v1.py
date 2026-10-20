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
 # Runtime proprio is float32; consumer promotes to float64 before norm.
 return float(np.linalg.norm(np.asarray(velocity,np.float32).astype(float)[:2]))

def meaningful_snapshot_equal(a,b):
 # Absolute artifact locations are audited by each arm seal, not geometric state.
 def normalized(s):
  return dict(s,records=[{k:v for k,v in r.items() if k!='input_sha256'} for r in s['records']])
 return normalized(a)==normalized(b)

def arm(name,cfg):
 D=H/('rolling_endpoint_navigation_e0_'+name+'_run_v1');status=json.loads((H/('rolling_endpoint_navigation_e0_'+name+'_status_v1.json')).read_text());assert status['status']=='finished' and status['child_returncode']==0
 seal=json.loads((D/'artifact_manifest_post_close.json').read_text());verify(seal['artifact_sha256']);assert json.loads((D/'candidate_envelope.json').read_text())==dict(arm=name,design=cfg)
 identity=json.loads((D/'identity.json').read_text());assert identity['complete'] and identity['known_GT_pose_control'] and identity['source_GT_pose_registration'] and identity['reset_count']==1 and not identity['internal_reset']
 ss=json.loads((D/'states.json').read_text());cc=json.loads((D/'physics_contacts.json').read_text());rr=json.loads((D/'policy_records.json').read_text());q=np.load(D/'requests.npz')['requests'];physics=dict(np.load(D/'physics.npz'));episode=json.loads((D/'task_episode.json').read_text())['actual_episode'];term=json.loads((D/'termination.json').read_text())
 assert identity['actions']==len(q) and identity['captures']==len(ss) and len(q)==len(rr) and np.array_equal(q,np.asarray([r['request'] for r in rr],np.float32))
 normalized=complete_execution_evidence(ss,cc,physics['simulation_time_s'],len(q));start=np.array(episode['start_position_world_m'][:2]);goal=np.array(episode['goal_position_world_m'][:2]);m=assess(ss,normalized,start,goal)
 assert m['outcome'] in ('success','collision','timeout'),m
 assert term['termination']==dict(success='success',collision='collision',timeout='timeout35')[m['outcome']]
 assert len(q)==350 or m['outcome'] in ('success','collision')
 assert all(r['known_GT_pose'] and not r['native_pose_used'] for r in rr)
 for row in rr:
  if not row.get('planner_act_called'):continue
  before=np.array(row['measured_speed_before_request'],np.float32);issued=np.array(row['request'],np.float32);assert np.array_equal(before[1::2],issued[1::2]);assert row['measured_speed_control_applied']==(not np.array_equal(before,issued))
  if row['measured_speed_control_eligible']:
   speed=reconstruct_current_speed(ss[row['frame']]['velocity']);assert abs(speed-row['measured_speed_worldXY_m_s'])<1e-6
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

def main():
 cfg=json.loads((H/'rolling_endpoint_navigation_e0_protocol_v1.json').read_text());verify(cfg['source_sha256']);verify(cfg['input_sha256']);a,ar=arm('b0',cfg);b,br=arm('paced',cfg);assert ar['episode']==br['episode']
 for k in ('camera_rig','pid_config','inner_config','tracker_MRO','frozen_source_method'):assert ar['identity'][k]==br['identity'][k],k
 assert br['identity']['policy_MRO'][1:]==ar['identity']['policy_MRO']
 events=[r['frame'] for r in br['rows'] if r.get('rolling_control_applied')];fork=events[0] if events else None;physical=requests=execution=images=geometry=False
 if fork is not None and fork<min(len(ar['requests']),len(br['requests'])):
  requests=np.array_equal(ar['requests'][:fork],br['requests'][:fork]);pa,pb=ar['physics'],br['physics'];ma=pa['simulation_time_s']<=fork*.1+1e-5;mb=pb['simulation_time_s']<=fork*.1+1e-5;physical=all(np.array_equal(pa[k][ma],pb[k][mb]) for k in pa if not k.startswith('raw_'))
  execution=shared_execution_prefix(ar['D'],br['D'],fork,len(ar['requests']),len(br['requests'])) and json.loads((ar['D']/'initial_execution_after_reset.json').read_text())==json.loads((br['D']/'initial_execution_after_reset.json').read_text())
  af={p.name:p for p in (ar['D']/'source_inputs').glob('*.npz') if int(p.stem.split('_')[-1])<=fork};bf={p.name:p for p in (br['D']/'source_inputs').glob('*.npz') if int(p.stem.split('_')[-1])<=fork};images=af.keys()==bf.keys() and all(all(np.array_equal(np.load(af[n])[k],np.load(bf[n])[k]) for k in ('left_rgb','right_rgb')) for n in af)
  geometry=all(meaningful_snapshot_equal(json.loads((ar['D']/'raw_point_memory_snapshots'/f'frame_{f:03d}.json').read_text()),json.loads((br['D']/'raw_point_memory_snapshots'/f'frame_{f:03d}.json').read_text())) for f in range(fork+1))
 gates=dict(actual_request_change=bool(events),no_contact=b['raw_contacts_zero'],arrival=b['success'],net_progress=b['along_start_goal_axis_m']>=a['along_start_goal_axis_m'],cross_axis_RMS=b['cross_axis']['rms_m']<=max(.2,1.2*a['cross_axis']['rms_m']),cross_axis_peak=b['cross_axis']['peak_m']<=max(.4,1.2*a['cross_axis']['peak_m']),latency=b['wall_median_s']<=1.5*a['wall_median_s'] and b['wall_p90_s']<=1.5*a['wall_p90_s'])
 r=dict(arms={'b0':a,'paced':b},gates=gates,taskH_candidate_accept=all(gates.values()),classification='conditional_TaskH_accept' if all(gates.values()) else 'measured_navigation_reject',first_intervention_frame=fork,intervention_count=len(events),prefix=dict(requests=requests,nonraw_physics=physical,execution=execution,RGB_pixels=images,raw_point_memory=geometry),causal_navigation_improvement=all(gates.values()) and requests and physical and execution and images and geometry,current_GT_pose_and_velocity_privileged=True,strict_cold_start_visual_admission=False,estimated_pose_result=None,learning_model_result=None,blindzone_full_3D_safety_not_proven=True,whole_mission_complete=False)
 p=H/'rolling_endpoint_navigation_e0_assessment_v1.json';assert not p.exists();p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r),flush=True)
if __name__=='__main__':
 try:main()
 except (AssertionError,ValueError,KeyError,OSError) as e:
  statuses={a:json.loads((H/('rolling_endpoint_navigation_e0_'+a+'_status_v1.json')).read_text()) if (H/('rolling_endpoint_navigation_e0_'+a+'_status_v1.json')).exists() else None for a in ('b0','paced')}
  execution_failure=any(s is None or s.get('status')!='finished' or s.get('child_returncode')!=0 for s in statuses.values());r=dict(classification='unassessed_execution' if execution_failure else 'unassessed_evidence',reason=repr(e),statuses=statuses,scientific_rejection=False,whole_mission_complete=False);p=H/'rolling_endpoint_navigation_e0_unassessed_v1.json';assert not p.exists();p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
