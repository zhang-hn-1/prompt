"""Historical scope decomposition, not a changed source gate or source pass."""
from pathlib import Path
import hashlib,json,numpy as np
from paired_current_evidence_v1 import qualify_packet
H=Path(__file__).resolve().parent;O=H/'existing_hazard_domain_historical_v1';O.mkdir();rows=[]
for case in ['moving_surface_A1_run_v1','paired_current_A2_run_v1']:
 D=H/case
 if not D.exists():
  matches=list(H.glob('*A1*run*'));raise AssertionError('case missing '+str(matches))
 for source in sorted((D/'sensor_packets').glob('source_*.npz')):
  f=int(source.stem.split('_')[-1]);p=dict(np.load(source));rgb=np.load(D/'online_inputs'/f'frame_{f:03d}.npz');mask=np.load(D/'masks'/f'frame_{f:03d}.npz');t=float(p['time_s']);q=qualify_packet(p,rgb['left_rgb'],rgb['right_rgb'],mask['left'],mask['right'],t,t,f);ix=np.flatnonzero(q['eligible']&(p['world'][:,2]>.05)&(p['world'][:,2]<=1.5));g=np.load(D/'offline_truth'/f'frame_{f:03d}.npz');ids=p['query_pixel_id'][ix];x,y=ids%640,ids//640;dep=g['left_depth_m'][y,x];valid=np.isfinite(dep)&(dep>0);rays=np.column_stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(ix))]);T=g['left_camera_usd_world'];world=T[:3,3]+(rays*dep[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;err=np.linalg.norm(world-p['world'][ix],axis=1);correct=valid&(err<=.1);rows.append(dict(case=case,frame=f,time_s=t,eligible_hazard_points=len(ix),valid_teacher_points=int(valid.sum()),within10cm=int(correct.sum()),precision_descriptive=None if not len(ix) else float(correct.mean()),role='existing hazard height domain only; not full source gate, coverage or nav'))
r=dict(scope='HISTORICAL_SOURCE_FAILURE_DOMAIN_DECOMPOSITION_NOT_NEW_GATE',rows=rows,original_height_domain=[.05,1.5],old_allpoints_reject_preserved=True,no_new_source_admission_or_control=True,code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(O/'assessment.json').write_text(json.dumps(r,indent=2)+'\n');non=[x for x in rows if x['eligible_hazard_points']];print('epochs',len(rows),'nonempty',len(non),'min descriptive precision',min(x['precision_descriptive'] for x in non),'frame35',[x for x in rows if x['case']=='paired_current_A2_run_v1' and x['frame']==35])
