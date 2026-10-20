from pathlib import Path
import json,hashlib,datetime,numpy as np
from audit_stereo import summarize
from stereo_geometry import FOCAL
H=Path(__file__).resolve().parent;O=H/'frozen_partial_original_scene_transfer_v1';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();seal=json.loads((O/'prediction_seal.json').read_text())
for f,s in seal['outputs'].items():assert sha(f)==s
rows=json.loads((O/'rows.json').read_text());groups={};scores=[]
for r in rows:
 for f,s in r['input_sha256'].items():assert sha(f)==s
 p=np.load(r['output']);gt=np.load(r['truth_file']);y,x=np.divmod(p['query_pixel_id'],640);z=gt['left_depth_m'][y,x];rays=np.column_stack(((x-319.5)/FOCAL,(y-239.5)/FOCAL,np.ones(len(x))));T=gt['left_camera_usd_world'];w=T[:3,3]+(rays*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T;valid=np.isfinite(z)&(z>0)&np.isfinite(w).all(axis=1);e=np.linalg.norm(p['world']-w,axis=1);e[~valid]=np.inf;band=valid&(w[:,2]>.05)&(w[:,2]<=1.5)
 groups.setdefault(r['sequence'],[]).append((e,band));scores.append(dict(sequence=r['sequence'],frame=r['frame'],n=len(e),metric=summarize(e)))
result=[]
for seq,data in groups.items():
 e=np.concatenate([a for a,b in data]);b=np.concatenate([b for a,b in data]);metric=summarize(e);checks=dict(n=len(e)>=100,metric95=metric['within_0p10m'] is not None and metric['within_0p10m']>=.95,p90=metric['unconditional_p90_m'] is not None and metric['unconditional_p90_m']<=.1,two100=sum(len(a)>=100 for a,b in data)>=2);result.append(dict(sequence=seq,checks=checks,geometry_decision='PASS_HISTORICAL_SELECTED_POINT_GEOMETRY_ONLY' if all(checks.values()) else 'REJECT_HISTORICAL_SELECTED_POINT_GEOMETRY',metric=metric,GT_height_band_precision=None if not len(b) else float(b.mean()),not_fullscene_semantic_gate=True))
a=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sequences=result,all_geometric_checks_pass=all(all(r['checks'].values()) for r in result),prediction_seal_sha256=sha(O/'prediction_seal.json'),not_fresh=True,GT_pose_upperbound=True,no_new_NN_or_physics=True,no_navigation_or_coverage_admission=True,old_dense_rejections_preserved=True,whole_complete=False)
(O/'assessment.json').write_text(json.dumps(a,indent=2)+'\n');(O/'scores.json').write_text(json.dumps(scores,indent=2)+'\n');print(json.dumps(a,indent=2))
