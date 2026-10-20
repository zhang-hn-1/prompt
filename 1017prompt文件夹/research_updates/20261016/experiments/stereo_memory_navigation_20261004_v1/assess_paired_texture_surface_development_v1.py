"""Development metric/risk comparison; no new filter variants or actor runs."""
from pathlib import Path
import json,hashlib,numpy as np
from audit_stereo import summarize
from stereo_geometry import FOCAL
from partial_point_risk_grid import PartialPointRiskGrid
from tentative_point_memory import TentativePointMemory
H=Path(__file__).resolve().parent;O=H/'paired_texture_surface_development_v1';s=json.loads((O/'prediction_seal_before_new_rule_labels.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
for group in ['inputs','outputs','source_sha256']:
 for f,d in s[group].items():assert sha(f)==d,f
result=[]
for out in [p for p in O.iterdir() if p.is_dir()]:
 D=H/out.name;es=[];cs=[];rows=json.loads((out/'rows.json').read_text())
 for r in rows:
  i=r['frame'];a=np.load(out/f'frame_{i:03d}.npz');g=np.load(D/'frames'/f'frame_{i:03d}.npz');q=a['query_pixel_id'];x,y=q%640,q//640;z=g['left_depth_m'][y,x];T=g['left_camera_usd_world'];rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))))
  with np.errstate(invalid='ignore'):w=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
  valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(1);e=np.linalg.norm(a['world']-w,axis=1);e[~valid]=np.inf;c=valid&(np.linalg.norm(w[:,:2],axis=1)<=.351)&(w[:,2]>.001)&(w[:,2]<=1.001);es.append(e);cs.append(c)
 e=np.concatenate(es);c=np.concatenate(cs);m=summarize(e);checks=dict(n100=len(e)>=100,metric95=m['within_0p10m']>=.95,p90=m['unconditional_p90_m']<=.1,class95=float(c.mean())>=.95,two100=sum(r['points']>=100 for r in rows)>=2);result.append(dict(case=out.name,checks=checks,world_metric=m,obstacle_precision=float(c.mean())))
D=H/'live_generic_height_navigation_run_v1';data=O/D.name;native=json.loads((D/'native_predictions.json').read_text());rows={r['frame']:r for r in json.loads((data/'rows.json').read_text())};grid=PartialPointRiskGrid(np.asarray(native[0]['position_world'])[:2]-20,10);memory=TentativePointMemory();risk=[]
for r in json.loads((D/'policy_records.json').read_text()):
 i=r['frame'];t=r['time_s']
 if i%5==0:
  a=np.load(data/f'frame_{i:03d}.npz');memory.update(dict(frame=i,time_s=t,world=a['world'],query_pixel_id=a['query_pixel_id'],partner_pixel_id=a['partner_pixel_id'],input_sha256={str(data/f'frame_{i:03d}.npz'):sha(data/f'frame_{i:03d}.npz')}),t)
 source=rows[(i//5)*5];grid.set_source(dict(frame=source['frame'],time_s=source['time_s'],records=memory.at(t)['records']));grid.update_points(np.empty((0,3)),np.asarray(native[i*5]['position_world']),t);xy=grid.origin+np.argwhere(grid.labels==1)*grid.resolution;compatible=np.linalg.norm(xy,axis=1)<=.35+np.sqrt(2)*grid.resolution/2+.1;risk.append(dict(frame=i,positive=len(xy),outside=int((~compatible).sum()),compatible=int(compatible.sum())))
b=json.loads((H/'generic_height_risk_cell_quality_v1/assessment.json').read_text());n=sum(r['outside'] for r in risk);riskcheck=dict(outside_time_pairs_smaller=n<b['outside_proxy_cell_time_pairs'],compatible_support_not_zero=sum(r['compatible'] for r in risk)>0);a=dict(decision='PASS_DEVELOPMENT_POINT_GATES_AND_RISK_COMPATIBILITY_REDUCTION_ONLY' if all(all(r['checks'].values()) for r in result) and all(riskcheck.values()) else 'REJECT_PAIRED_TEXTURE_SUPPORT_DEVELOPMENT',sequences=result,risk_checks=riskcheck,G_old_outside=b['outside_proxy_cell_time_pairs'],G_new_outside=n,G_new_positive=sum(r['positive'] for r in risk),G_peak_outside=max(r['outside'] for r in risk),closed_all_DEVELOPMENT_NOT_FRESH=True,no_online_or_nav_admission=True,old_generic_pose_navigation_rejects_preserved=True,no_NCC_or_std_search=True)
(O/'G_risk_rows.json').write_text(json.dumps(risk,indent=2)+'\n');(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');print(json.dumps(a,indent=2))
