"""Independent frozen-reference and real XY/contact metrics; no policy replay/refit."""
from pathlib import Path
import json,hashlib,numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from baseline35_metrics_v1 import assess
from matched_progress_v1 import matched_progress
H=Path(__file__).resolve().parent;D=H/'endpoint_tail_e1_run_v1';B=H/'native_pose_baseline35_e1_coverage_run_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify(d):
 m=json.loads((d/'artifact_manifest_post_close.json').read_text());assert all(sha(p)==h for p,h in m['artifact_sha256'].items());return len(m['artifact_sha256'])
def geometric(xy,ref):
 delta=np.diff(ref,axis=0);length=np.linalg.norm(delta,axis=1);assert np.all(length>0);arc=np.r_[0.,np.cumsum(length)];u=np.clip(((xy[:,None]-ref[:-1])*delta).sum(2)/length**2,0,1);project=ref[:-1]+u[:,:,None]*delta;distance=np.linalg.norm(xy[:,None]-project,axis=2);j=distance.argmin(1);cross=distance[np.arange(len(xy)),j];progress=arc[j]+u[np.arange(len(xy)),j]*length[j];return cross,progress

def phase_metrics(states,rows,ref,cut):
 intervals=[i for i,r in enumerate(rows) if r.get('planner_act_called') and r.get('local_reference_version')==1]
 indices=sorted({j for i in intervals for j in (i,i+1) if j<len(states) and states[j]['time_s']<=cut+1e-5});assert indices
 xy=np.asarray([states[i]['position'][:2] for i in indices]);cross,progress=geometric(xy,ref);enddist=np.linalg.norm(xy-ref[-1],axis=1)
 return dict(first_time_s=float(states[indices[0]]['time_s']),last_time_s=float(states[indices[-1]]['time_s']),samples=len(indices),cross_RMS_m=float(np.sqrt(np.mean(cross**2))),cross_peak_m=float(cross.max()),real_max_projected_arc_m=float(progress.max()),real_endpoint_min_distance_m=float(enddist.min()),real_endpoint_reached=bool(np.any(enddist<=.2)),not_from_policy_cursor=True),xy

def main():
 st=json.loads((H/'endpoint_tail_e1_status_v1.json').read_text());assert st['status']=='finished' and st['exit_code']==0;raw=verify(D);verify(B);cfg=json.loads((H/'endpoint_tail_e1_protocol_v1.json').read_text());assert all(sha(p)==h for p,h in cfg['source_sha256'].items());assert all(sha(p)==h for p,h in cfg['input_sha256'].items());f=json.loads((H/'endpoint_tail_frozen_B0_reference_v1.json').read_text());ref=np.asarray(f['reference_xy']);s=json.loads((D/'states.json').read_text());bs=json.loads((B/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text());brows=json.loads((B/'policy_records.json').read_text());contacts=json.loads((D/'physics_contacts.json').read_text());ep=json.loads((D/'task_episode.json').read_text())['actual_episode'];requests=np.load(D/'requests.npz')['requests'];br=np.load(B/'requests.npz')['requests'];p=np.load(D/'physics.npz');bp=np.load(B/'physics.npz');assert len(s)==len(requests)+1 and len(p['simulation_time_s'])==len(requests)*5+1
 r=assess(s,[dict(time_s=a['simulation_time_s'],collision=a['contact'],available=a['available']) for a in contacts],ep['start_position_world_m'][:2],ep['goal_position_world_m'][:2]);base=json.loads((B/'native_pose_assessment_v2.json').read_text());phase,pxy=phase_metrics(s,rows,ref,r['scored_end_s']);bphase,bxy=phase_metrics(bs,brows,ref,base['scored_end_s']);events=[dict(frame=a['frame'],time_s=a['time_s'],endpoint_distance_m=a['endpoint_tail_this_call']['endpoint_distance_m'],request=a['request'][:2]) for a in rows if a.get('endpoint_tail_this_call') and a['endpoint_tail_this_call']['endpoint_tail_intervened']];n=min(len(requests),len(br));diff=np.flatnonzero(np.any(requests[:n]!=br[:n],axis=1));first=int(diff[0]) if len(diff) else None;length=(first if first is not None else n)*5+1;prefix={k:np.array_equal(p[k][:length],bp[k][:length]) for k in p.files if not k.startswith('raw_') and k not in ('action_block_index','physics_step_in_block')}
 a=np.load(D/'source_inputs/frame_000.npz');b=np.load(B/'source_inputs/frame_000.npz');rgb={eye:int(np.count_nonzero(a[eye+'_rgb']!=b[eye+'_rgb'])) for eye in ('left','right')};gain=r['along_start_goal_axis_m']-base['along_start_goal_axis_m'];checks=dict(contact_available_no_hit=r['contacts_all_available'] and r['contact_time_s'] is None,actual_frozen_endpoint_reached=phase['real_endpoint_reached'],axis_progress_gain_at_least_0p5=gain>=.5,first_ref_RMS_no_worse=phase['cross_RMS_m']<=bphase['cross_RMS_m'],first_ref_peak_no_worse=phase['cross_peak_m']<=bphase['cross_peak_m'],actual_consumer_intervened=bool(events),request_changed_relative_actual_B0=first is not None);passed=all(checks.values())
 videos={}
 for tag in ('stereo_view','third_person'):
  cap=cv2.VideoCapture(str(D/(tag+'.mp4')));fps=cap.get(cv2.CAP_PROP_FPS);count=0
  while True:
   ok,_=cap.read()
   if not ok:break
   count+=1
  cap.release();assert count==len(s) and abs(fps-10)<1e-6;videos[tag]=dict(decoded_frames=count,fps=fps)
 start=np.asarray(ep['start_position_world_m'][:2]);goal=np.asarray(ep['goal_position_world_m'][:2]);matched=matched_progress(s,bs,min(r['scored_end_s'],base['scored_end_s']),start,(goal-start)/np.linalg.norm(goal-start))
 r.update(matched_time_axis_progress=matched,terminal_gain_not_same_clock_if_collision=True,local_control_candidate_pass=passed,gate_checks=checks,decision='local gate pass, formal goal scored separately' if passed else 'sole local control candidate rejected; no retry/threshold change',first_reference_phase=phase,B0_first_reference_phase=bphase,final_axis_gain_m=gain,intervention_events=events,first_request_difference_frame=first,nonraw_physics_before_request_fork_exact=prefix,initial_RGB_different_values=rgb,baseline_is_actual_historical=True,fullcausal_same_RGB_AB=False,no_unknown_as_free=True,no_worldmodel_refit=True,source_accuracy_not_admission=True,whole_mission_complete=False,raw_sealed_items_checked=raw,videos=videos)
 xy=np.asarray([a['position'][:2] for a in s]);oldxy=np.asarray([a['position'][:2] for a in bs]);fig,axes=plt.subplots(1,2,figsize=(11,4));axes[0].plot(oldxy[:,0],oldxy[:,1],label='actual historical B0');axes[0].plot(xy[:,0],xy[:,1],label='actual tail trial');axes[0].plot(ref[:,0],ref[:,1],'k--',label='frozen B0 first reference');axes[0].scatter(*ep['goal_position_world_m'][:2],c='red');axes[0].legend();axes[0].set_aspect('equal');goal=np.asarray(ep['goal_position_world_m'][:2]);axes[1].plot([a['time_s'] for a in bs],np.linalg.norm(oldxy-goal,axis=1),label='B0');axes[1].plot([a['time_s'] for a in s],np.linalg.norm(xy-goal,axis=1),label='tail trial');axes[1].set(xlabel='simulation time(s)',ylabel='actual goal distance(m)');axes[1].legend();fig.tight_layout();fig.savefig(D/'endpoint_tail_actual_trajectory_v1.png',dpi=150);plt.close(fig);(D/'endpoint_tail_assessment_v1.json').write_text(json.dumps(r,indent=2)+'\n');(D/'artifact_manifest_post_assessment_v1.json').write_text(json.dumps(dict(artifact_sha256={str(p):sha(p) for p in D.rglob('*') if p.is_file()}),indent=2)+'\n');print(json.dumps(r,indent=2))
if __name__=='__main__':main()
