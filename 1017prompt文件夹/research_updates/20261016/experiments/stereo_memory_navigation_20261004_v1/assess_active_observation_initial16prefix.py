"""Complete-sequence motion and AFTER-PREDICTION limited depth-label assessment."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np
import cv2
from assess_feature_tracks import label
from guarded_response import yaw_q
from route_geometry_audit import project
H=Path(__file__).resolve().parent;root=H/'active_observation_run';out=H/'active_feature_runs';sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
    p=json.loads((H/'active_observation_protocol.json').read_text());m=json.loads((out/'manifest.json').read_text());assert m['labels_not_loaded'] and len(m['rows'])==181
    for f,s in p['source_sha256'].items():assert sha(f)==s,f
    for row in m['rows']:assert sha(out/f"prediction_{row['frame']:03d}.json")==row['sha256']
    stats={arm:dict(queries=0,retained=0,correct=0,labelled_queries=0,labelled_retained=0,past=0,conflicts=0,errors=[]) for arm in ['current','causal']};rows=[];labels_by_frame={};points=[];memory_rows=[]
    for i in range(181):
        r=json.loads((out/f'prediction_{i:03d}.json').read_text());raw=np.load(root/'offline_truth'/f'frame_{i:03d}.npz');known=np.load(root/'known_poses'/f'frame_{i:03d}.npz')
        for eye in ['left','right']:np.testing.assert_equal(raw[eye+'_camera_usd_world'],known[eye+'_camera_usd_world'])
        labels=[label(xy,raw,'left') for xy in r['current']['query_xy']];labels_by_frame[i]=labels;correct_by_arm={}
        for arm in ['current','causal']:
            pred=r[arm];assert pred['query_xy']==r['current']['query_xy'];s=stats[arm];s['queries']+=len(labels);s['labelled_queries']+=sum(pt is not None for pt,k in labels);s['retained']+=len(pred['points']);s['conflicts']+=pred['conflicting_queries'];errs=[];correct=set();past=0
            for pt in pred['points']:
                j=pt['query'][2];truth,kind=labels[j];e=None if truth is None else float(np.linalg.norm(np.array(pt['world'])-truth));ok=e is not None and e<=.1;was_past=pt['source'][3]<r['time_s'];past+=was_past
                if e is not None:errs.append(e);s['errors'].append(e);s['labelled_retained']+=1
                if ok:s['correct']+=1;correct.add(j)
                points.append(dict(frame=i,arm=arm,index=j,error_m=e,correct=ok,label_kind=kind,source=pt['source'],past=bool(was_past),label_world=None if truth is None else truth.tolist(),world=pt['world'],sigma_m=pt['sigma_m']))
            s['past']+=past;correct_by_arm[arm]=correct
            kinds={}
            for pt,k in labels:kinds[k]=kinds.get(k,0)+1
            rows.append(dict(frame=i,time_s=r['time_s'],arm=arm,queries=len(labels),label_kinds=kinds,retained=len(pred['points']),correct=len(correct),past=past,conflicts=pred['conflicting_queries'],p90_m=None if not errs else float(np.percentile(errs,90))))
        rows[-1].update(added_correct_vs_current=len(correct_by_arm['causal']-correct_by_arm['current']),lost_correct_vs_current=len(correct_by_arm['current']-correct_by_arm['causal']))
        valid=got=0
        for pt in r['memory']:
            truth,kind=labels_by_frame[pt['query'][0]][pt['query'][2]]
            if truth is not None:valid+=1;got+=np.linalg.norm(np.array(pt['world'])-truth)<=.1
        memory_rows.append(dict(frame=i,records=len(r['memory']),labelled_records=valid,correct_records=int(got),older_queries=sum(pt['query'][3]<r['time_s'] for pt in r['memory']),certified_free=False))
    for arm,s in stats.items():
        errs=s.pop('errors');s['unconditional_fraction']=s['correct']/s['queries'];s['conditional_fraction']=s['correct']/s['retained'] if s['retained'] else None;s['p90_m']=None if not errs else float(np.percentile(errs,90));s['gate_pass']=bool(s['queries']>=100 and s['unconditional_fraction']>=.9 and s['p90_m'] is not None and s['p90_m']<=.1)
    ph=np.load(root/'physics.npz');old=np.load(H/'large_bank_runs/positive/physics.npz');entry=next(e for e in json.loads((H/'guarded_response_library.json').read_text()) if e['name']=='positive');decision=json.loads((root/'decision.json').read_text());current=decision['current'];yaw=current['yaw'];c,s=np.cos(yaw),np.sin(yaw);reference=np.asarray(entry['xy'])@np.array([[c,-s],[s,c]]).T+current['position'][:2];actual=ph['position_world_xy'][300:];error=np.linalg.norm(reference-actual,axis=1)
    q0=old['root_quaternion_world_wxyz'];q1=ph['root_quaternion_world_wxyz'];bank=lambda q:np.arcsin(np.clip(2*(q[0]*q[1]+q[2]*q[3])/np.dot(q,q),-1,1))
    rows_motion=[]
    for t in p['motion_gate']['horizons_after_motion_start_s']:
        k=300+round(t*50);dyaw=(yaw_q(q1[k])-current['yaw'])-(yaw_q(q0[k])-entry['state']['yaw']);dyaw=np.arctan2(np.sin(dyaw),np.cos(dyaw));dbank=np.degrees(bank(q1[k])-bank(q0[k]));rows_motion.append(dict(after_motion_s=t,xy_error_m=float(error[k-300]),yaw_error_rad=float(abs(dyaw)),bank_error_deg=float(abs(dbank))))
    def first_low(physics):
        velocity=np.linalg.norm(physics['linear_velocity_world_m_s'][::5,:2],axis=1);count=0
        for i in range(120,len(velocity)):
            count=count+1 if velocity[i]<.05 else 0
            if count>=5:return i*.1
        return None
    qual=first_low(ph);oldqual=first_low(old);contacts=json.loads((root/'physics_contacts.json').read_text());collision=any(r['contact'] for r in contacts)
    motion_pass=bool(error.max()<=.1 and all(r['yaw_error_rad']<=.15 and r['bank_error_deg']<=5 for r in rows_motion) and qual is not None and oldqual is not None and abs(qual-oldqual)<=.3 and not collision)
    path=reference[np.r_[True,np.linalg.norm(np.diff(reference,axis=0),axis=1)>0]];progress,lateral,_=project(actual,path)
    prefix=np.load(H/'online_bank_hold_navigation_run/physics.npz');prefix_diffs={k:float(np.max(np.abs(ph[k][:301]-prefix[k][:301]))) for k in ph.files if not k.startswith('raw_')}
    states=json.loads((root/'execution_states.json').read_text());oldstates=json.loads((H/'online_bank_hold_navigation_run/execution_states.json').read_text());exec_prefix=states[:61]==oldstates[:61]
    source_inputs=H/'online_bank_hold_navigation_run/online_inputs';history_max=0.
    for i in range(61):
        a=np.load(root/'history'/f'frame_{i:03d}.npz');b=np.load(source_inputs/f'frame_{i:03d}.npz')
        for k in a.files:history_max=max(history_max,float(np.max(np.abs(a[k]-b[k]))))
    video=[]
    for name in ['stereo_view','third_person']:
        cap=cv2.VideoCapture(str(root/(name+'.mp4')));n=0
        while True:
            ok,frame=cap.read()
            if not ok:break
            n+=1
        fps=float(cap.get(cv2.CAP_PROP_FPS));cap.release();video.append(dict(name=name,decoded_frames=n,fps=fps));assert n==181 and fps==10
    clocks=json.loads((root/'camera_clocks.json').read_text());age=max(abs(r['raw_time_s']-t) for r in clocks for t in r['eyes'].values())
    result=dict(assessed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),predictions_saved_utc=m['saved_utc'],source_stats=stats,rows=rows,points=points,memory_rows=memory_rows,
                motion=dict(gate_pass=motion_pass,xy_rms_m=float(np.sqrt(np.mean(error**2))),xy_peak_m=float(error.max()),horizons=rows_motion,first_five_low_s=qual,reference_first_five_low_s=oldqual,physical_contact=collision,contact_samples=len(contacts),along_measured_reference_progress_m=float(progress[-1]-progress[0]),along_measured_reference_length_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),lateral_rms_m=float(np.sqrt(np.mean(lateral**2))),lateral_peak_m=float(lateral.max()),root_actual_xy_path_m=float(np.linalg.norm(np.diff(actual,axis=0),axis=1).sum()),root_xy_net_m=(actual[-1]-actual[0]).tolist(),no_navigation_success_claim=True),
                prefix=dict(nonraw_field_count=len(prefix_diffs),nonraw_max_differences=prefix_diffs,complete_execution_exact=exec_prefix,history_max_difference=history_max,raw_time_origin_delta_s=float(ph['raw_simulation_time_s'][0]-prefix['raw_simulation_time_s'][0]),raw_step_origin_delta=int(ph['raw_physics_step_index'][0]-prefix['raw_physics_step_index'][0])),
                acquisition=dict(requests=len(np.load(root/'requests.npz')['requests']),captures=len(clocks),physics_samples=len(ph['simulation_time_s']),max_camera_clock_age_s=age,video=video),source_candidate_accepted=stats['causal']['gate_pass'],scope='known-map guarded actualoriginalE1 observation, prospective whole sameoldscene; current nearestpixel limitedlabels not independentbodyID/fullsurface certificate; sparsememory notfree or nav')
    (out/'assessment.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['rows','points','memory_rows']},indent=2))
if __name__=='__main__':main()
