from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent;D=H/'E1_continuous_tube_navigation_run_v1'
p=json.loads((D/'policy_records.json').read_text());s=json.loads((D/'states.json').read_text());shadow=json.loads((D/'B0_shadow_until_fork.json').read_text());phys=np.load(D/'physics.npz');xy=np.asarray([x['position'][:2] for x in s]);req=np.asarray([x['request'] for x in p]);time=np.asarray([x['time_s'] for x in s])
def canon(x): return np.asarray([] if x is None else x,float).reshape(-1,2)
def projection(points,ref):
 a=ref[:-1];v=np.diff(ref,axis=0);l=np.linalg.norm(v,axis=1);den=(v*v).sum(axis=1);arc=np.r_[0,np.cumsum(l)];ds=[];progress=[]
 for q in points:
  f=np.divide(((q-a)*v).sum(axis=1),den,out=np.zeros(len(v)),where=den>0).clip(0,1);d=np.linalg.norm(q-(a+f[:,None]*v),axis=1);j=int(np.argmin(d));ds.append(float(d[j]));progress.append(float(arc[j]+f[j]*l[j]))
 return np.array(ds),np.array(progress)
forks=[r['frame'] for r in shadow if r['first_request_fork']];assert len(forks)==1 and len(shadow)==forks[0]+1;f=forks[0];assert all(r['same_request'] for r in shadow[:-1]);assert not shadow[-1]['same_request']
refdiff=[r['frame'] for r in shadow if not np.array_equal(canon(r['B0_reference']),canon(p[r['frame']]['installed_reference_xy_after_act']))]
versions=[0]+[i for i in range(1,len(p)) if p[i]['local_reference_version']!=p[i-1]['local_reference_version']]+[len(p)];segments=[]
for begin,end in zip(versions[:-1],versions[1:]):
 ref=canon(p[begin]['installed_reference_xy_after_act'])
 if len(ref)<2:continue
 d,pr=projection(xy[begin:end+1],ref);rr=req[begin:end];segments.append(dict(begin=begin,end=end,time_start=float(time[begin]),time_end=float(time[end]),reference_version=p[begin]['local_reference_version'],frozen_reference_length_m=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum()),true_projected_start_m=float(pr[0]),true_projected_end_m=float(pr[-1]),net_projected_progress_m=float(pr[-1]-pr[0]),cross_track_rms_m=float(np.sqrt(np.mean(d*d))),cross_track_peak_m=float(max(d)),enter_error_m=float(d[0]),exit_error_m=float(d[-1]),samples_outside_existing_empirical_0p2m=int(sum(d>.2)),first_outside_frame=(begin+int(np.flatnonzero(d>.2)[0])) if np.any(d>.2) else None,aux_protocol_boundary_request_fraction=float(np.mean(abs(rr[:,1])>=np.deg2rad(30)-1e-7)),direction=p[begin]['tracker_state_after_act']['active_direction']))
# Common prefix applied discrepancy is measured directly from online independent B0 request, not later counterfactual states.
raw=phys['raw_command_target'];apply=phys['applied_action'];smooth=phys['smoothed_target'];blocks=phys['action_block_index'];ticks=np.flatnonzero(blocks==f);assert len(ticks)>0
# Applied actions cannot be compared with missing B0 physics after fork. Save actual transition and preceding target only.
actual_at_fork={k:phys[k][ticks].tolist() for k in ['raw_command_target','clipped_target','smoothed_target','applied_action','joint_position_rad','joint_velocity_rad_s','physics_step_in_block']}
common=canon(shadow[-1]['B0_reference']);cd,cp=projection(xy[f:],common)
lat=np.asarray([r['end_to_end_wall_s'] for r in p]);out=dict(scope=json.loads((H/'continuous_tube_tracking_audit_protocol_v1.json').read_text()),first_reference_difference_frame=min(refdiff) if refdiff else None,reference_difference_count_before_request_fork=len(refdiff),first_request_fork_frame=f,first_request_fork_time_s=float(time[f]),B0_request_at_fork=shadow[-1]['B0_request'],candidate_request_at_fork=shadow[-1]['candidate_request'],shadow_stops_exactly_at_first_fork=True,no_B0_physical_truth_after_fork=True,actual_execution_at_fork=actual_at_fork,segments=segments,common_fork_reference_candidate_only=dict(cross_track_rms_m=float(np.sqrt(np.mean(cd*cd))),cross_track_peak_m=float(max(cd)),net_projected_progress_m=float(cp[-1]-cp[0]),scope='descriptive frozen B0 geometry only; NOT B0 alternate-plan true labels'),latency=dict(median_wall_s=float(np.median(lat)),p90_wall_s=float(np.quantile(lat,.9)),max_wall_s=float(max(lat)),fraction_over_0p1s=float(np.mean(lat>.1))),navigation_improvement_accepted=False,formal35_NOTRUN=True)
(D/'tracking_execution_audit_v1.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k not in ['scope','actual_execution_at_fork','segments']}));print(json.dumps(segments))
