from pathlib import Path
import json,hashlib,numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1];R=W/'experiments/viewpoint_confirmation_navigation_20261012_v1/candidate_e2_run';seal=json.loads((D/'prediction_seal.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for group in ['input_sha256','output_sha256']:
 for p,h in seal[group].items():assert sha(p)==h,p
errors={'before':[],'after':[]};totals=dict(before_obstacle=0,after_obstacle=0,before_floor=0,after_floor=0,removed=0,original_pixels=0,new_pixels=0);rows=[]
for p in sorted((D/'qualified_predictions').glob('*.npz')):
 a=np.load(p);b=np.load(R/'source_predictions'/p.name);f=np.load(R/'frames'/p.name);gt=f['left_depth_m'];z=b['depth_m'];valid=b['confidence']&np.isfinite(z)&(z>=.5)&(z<=3)&np.isfinite(gt)&(gt>0);new=a['confidence']&valid
 np.testing.assert_array_equal(a['confidence'],b['confidence']&~a['ground_ambiguous']);assert not np.any(a['confidence']&~b['confidence'])
 q=np.flatnonzero(valid);x=q%640;y=q//640;T=f['left_camera_usd_world'];rays=np.column_stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))]);world=T[:3,3]+(rays*gt.ravel()[q,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;keep=new.ravel()[q];floor=abs(world[:,2])<=.03;obstacle=(world[:,2]>.05)&(world[:,2]<=1.5)
 for label,k in [('before',valid),('after',new)]:errors[label].append(abs(z[k]-gt[k]))
 for key,v in [('before_obstacle',obstacle.sum()),('after_obstacle',(obstacle&keep).sum()),('before_floor',floor.sum()),('after_floor',(floor&keep).sum()),('removed',a['ground_ambiguity_count']),('original_pixels',valid.sum()),('new_pixels',new.sum())]:totals[key]+=int(v)
 rows.append(dict(frame=p.stem,removed=int(a['ground_ambiguity_count'])))
metrics={}
for k,v in errors.items():
 e=np.concatenate(v);metrics[k]=dict(pixels=len(e),p90=float(np.quantile(e,.9)) if len(e) else None,outlier_fraction=float((e>.3).mean()) if len(e) else None)
r=totals['after_obstacle']/totals['before_obstacle'] if totals['before_obstacle'] else 0;a=metrics['after'];accuracy=a['pixels']>=1000 and a['p90']<=.15 and a['outlier_fraction']<=.1;retention=r>=.95
x=dict(metrics=metrics,totals=totals,real_obstacle_pixel_retention=r,accuracy_pass=bool(accuracy),conservative_retention_pass=bool(retention),source_entry_qualified=bool(accuracy and retention),actual_navigation_reserved=0,scope='Only actual181pairs development qualification. Same-frame renderer labels scored after predictionseal, not inference inputs. Correlated pixels not independent samples; groundplane prior and matching ambiguity not free volume or reliable navigation.')
(D/'assessment.json').write_text(json.dumps(x,indent=2)+'\n');print(json.dumps(x))
