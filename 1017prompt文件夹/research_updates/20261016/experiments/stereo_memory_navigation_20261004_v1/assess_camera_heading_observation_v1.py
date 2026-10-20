"""Afterwholeclose and requestreconstruction, grade ALL frozen probe sourceupdates."""
from pathlib import Path
import datetime,json
import cv2,numpy as np
H=Path(__file__).resolve().parent;D=H/'camera_heading_observation_run_v1'
verified=json.loads((H/'camera_heading_observation_interface_verification_v1.json').read_text())
assert not verified['renderer_labels_read'] and verified['actualfinal_request_reconstruction_exact']
event=verified['event_time_s'];rows=[]
qrows=json.loads((D/'hazard_qualification_records.json').read_text())
seed=np.load(D/'sensor_packets/source_180.npz')['world'];seed=seed[(seed[:,2]>.05)&(seed[:,2]<=1.5)]
seedbins=set(map(tuple,np.floor(seed[:,:2]/.1).astype(int)))
for r in qrows:
    t=r['time_s']
    if event is None or not(event<t<=event+6+1e-6):continue
    i=r['frame'];f=np.load(D/'sensor_packets'/f'source_{i:03d}.npz');w=f['world'];haz=(w[:,2]>.05)&(w[:,2]<=1.5);w=w[haz]
    gt=np.load(D/'offline_truth'/f'frame_{i:03d}.npz');errors=[];actual=[]
    for eye,k in [('left','query_pixel_id'),('right','partner_pixel_id')]:
        idx=f[k][haz];x,y=idx%640,idx//640;z=gt[eye+'_depth_m'][y,x];T=gt[eye+'_camera_usd_world']
        ray=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(x))]
        with np.errstate(invalid='ignore'):
            truth=T[:3,3]+(ray*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
            actual.append(truth);errors.append(np.linalg.norm(truth-w,axis=1))
    e=np.asarray(errors).T;finite=np.isfinite(e).all(1);ok=finite&(e<=.1).all(1)
    bins=np.floor(w[:,:2]/.1).astype(int);new=np.array([tuple(b) not in seedbins for b in bins],bool);newok=new&ok
    truebins=np.floor(actual[0][ok,:2]/.1).astype(int);truenew=np.array([tuple(b) not in seedbins for b in truebins],bool)
    rows.append(dict(frame=i,time_s=t,n=len(w),correct=int(ok.sum()),insufficient=int((~finite).sum()),
                     finite_wrong=int((finite&~ok).sum()),precision=float(ok.mean()) if len(w) else None,
                     correct_new_points=int(newok.sum()),correct_new_bins=len(set(map(tuple,bins[newok]))),
                     supplemental_true_new_points=int(truenew.sum()),supplemental_true_new_bins=len(set(map(tuple,truebins[truenew])))))
nonempty=[r for r in rows if r['n']];precision=bool(nonempty) and all(r['precision']>=.95 for r in nonempty)
size=sum(r['n']>=100 for r in rows)>=2
coverage=any(r['correct_new_points']>=20 and r['correct_new_bins']>=2 for r in rows)
contacts=json.loads((D/'physics_contacts.json').read_text())
windowcontacts=[r for r in contacts if event is not None and event<=r['simulation_time_s']<event+6]
no_contact=not any(r['contact'] for r in windowcontacts)
histories=sorted((D/'history').glob('frame_*.npz'));bank=[];states=json.loads((D/'states.json').read_text())
for p in histories:
    i=int(p.stem.split('_')[-1]);t=states[i]['time_s']
    if event is not None and event-.1<=t<=event+6+.1:
        g=np.load(p)['proprio_history'][-1,6:9];bank.append(dict(frame=i,time_s=t,public_gravity_bank_rad=float(np.arctan2(g[1],-g[2]))))
videos=[]
for name in ('stereo_view','third_person'):
    video=cv2.VideoCapture(str(D/(name+'.mp4')));count=0
    while True:
        ok,_=video.read()
        if not ok:break
        count+=1
    fps=video.get(cv2.CAP_PROP_FPS);video.release();videos.append(dict(name=name,frames=count,fps=fps))
metrics=json.loads((D/'metrics.json').read_text())
probe_rows=[r['local'] for r in json.loads((D/'policy_records.json').read_text()) if event is not None and event<=r['local']['time_s']<event+6]
turn_exercised=bool(probe_rows) and probe_rows[0]['visibility_probe']['event_camera_heading_rad'] is not None
heading_reached=any(r['visibility_probe']['camera_heading_goal_reached'] for r in probe_rows)
if event is not None:
 event_frame=verified['event_frame'];event_xy=np.array(states[event_frame]['position'][:2]);window_states=[s for s in states if event<=s['time_s']<=event+6+1e-6];max_displacement=max((float(np.linalg.norm(np.array(s['position'][:2])-event_xy)) for s in window_states),default=0.)
else:max_displacement=0.
passed=turn_exercised and heading_reached and max_displacement>=.05 and event is not None and verified['changed_final_requests']>0 and no_contact and precision and size and coverage
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='NOT_EXERCISED_TURN' if not turn_exercised else ('PASS_LIMITED_ACTUAL_SENSING_ONLY' if passed else 'REJECT_CAMERA_HEADING_OBSERVATION_PROBE'),
            turn_exercised=turn_exercised,camera_heading_goal_reached_within_observation=heading_reached,max_actual_observation_displacement_m=max_displacement,event_time_s=event,probe_end_time_s=event+6 if event is not None else None,
            final_requests_changed=verified['changed_final_requests'],no_contact_during_probe=no_contact,
            every_nonempty_geometry_ge95=precision,two_distinct_100point_updates=size,newregion_20points_2bins=coverage,
            all_window_source_updates=rows,bank_public_gravity_trace=bank,videos=videos,
            actual_task_metrics_separate_not_formal_navigation=metrics,whole_research_complete=False,
            knownpose_privileged=True,original35s_rule_unchanged=True,no_navigation_SR_or_global_source_admission=True,
            no_retry_or_duration_angle_threshold_model_scan=True,steady_point_not_transient_model_admission=True)
out=H/'camera_heading_observation_assessment_v1.json';assert not out.exists();out.write_text(json.dumps(result,indent=2)+'\n')
print(result['decision']);print(json.dumps(rows,indent=2));print('task',metrics);print('videos',videos)
