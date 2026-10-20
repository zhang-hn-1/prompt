from pathlib import Path
import json,hashlib,numpy as np
from audit_stereo import summarize
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;D=H/'live_partial_point_navigation_run_v1';O=H/'generic_depth_height_source_development_v1';seal=json.loads((O/'prediction_seal_pretruth.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for group in ('inputs','outputs'):
 for f,s in seal[group].items():assert sha(f)==s,f
es=[];cs=[];rows=[]
for r in json.loads((O/'rows.json').read_text()):
 i=r['frame'];a=np.load(O/'points'/f'frame_{i:03d}.npz');gt=np.load(D/'frames'/f'frame_{i:03d}.npz');y,x=np.divmod(a['query_pixel_id'],640);z=gt['left_depth_m'][y,x];rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))));T=gt['left_camera_usd_world']
 with np.errstate(invalid='ignore'):w=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(-1);e=np.linalg.norm(a['world']-w,axis=1);e[~valid]=np.inf;c=valid&(np.linalg.norm(w[:,:2],axis=1)<=.351)&(w[:,2]>.001)&(w[:,2]<=1.001);ground=valid&(np.abs(w[:,2])<=.001);selfpart=valid&(np.linalg.norm(w-gt['root_position_world_m'],axis=1)<=.55)
 es.append(e);cs.append(c);rows.append(dict(frame=i,n=len(e),obstacle=int(c.sum()),true_ground=int((ground&~c).sum()),body_near_root=int((selfpart&~c&~ground).sum()),other_valid=int((valid&~c&~ground&~selfpart).sum()),invalid=int((~valid).sum())))
e=np.concatenate(es);c=np.concatenate(cs);m=summarize(e);checks=dict(n100=len(e)>=100,metric95=m['within_0p10m']>=.95,p90=m['unconditional_p90_m']<=.1,class95=float(c.mean())>=.95,two100=sum(r['n']>=100 for r in rows)>=2);a=dict(decision='PASS_GENERIC_HEIGHT_SOURCE_DEVELOPMENT_ONLY' if all(checks.values()) else 'REJECT_GENERIC_HEIGHT_SOURCE_DEVELOPMENT',checks=checks,world_metric=m,obstacle_precision=float(c.mean()),false_positive_causes={k:sum(r[k] for r in rows) for k in ('true_ground','body_near_root','other_valid','invalid')},frames=rows,not_fresh=True,no_online_generic_source_or_nav=True,no_new_threshold_or_candidate=True,old_M_partial_nav_and_source_pass_unchanged=True,whole_complete=False);(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps({k:v for k,v in a.items() if k!='frames'},indent=2))
