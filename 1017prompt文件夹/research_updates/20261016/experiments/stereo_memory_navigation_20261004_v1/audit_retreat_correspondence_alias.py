"""Fixed four packet groups, measured GT correspondence diagnosis only."""
from pathlib import Path
import json,numpy as np,hashlib,datetime
H=Path(__file__).resolve().parent;o=H/'retreat_observation_run';groups=[('Fast',i,o/'sensor_packets'/f'source_{i:03d}.npz') for i in [120,180]]+[('frozenSGBM',i,H/'retreat_frozen_sgbm_probe_v2'/f'packet_{i:03d}.npz') for i in [120,180]];inputs=[Path(__file__).resolve(),H/'retreat_visibility_diagnostic.json']+[p for _,_,p in groups]
for i in [120,180]:inputs.extend([o/'offline_truth'/f'frame_{i:03d}.npz',o/'online_inputs'/f'frame_{i:03d}.npz'])
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();seal=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),input_sha256={str(p):sha(p) for p in inputs},scope='four presealed packet groups; geometric instance proxy and photometric diagnostics only, not renderer ID or new confidence gate')
(H/'retreat_correspondence_alias_preanalysis.json').write_text(json.dumps(seal,indent=2)+'\n');centres=np.array(json.loads((H/'retreat_visibility_diagnostic.json').read_text())['divider_centres']);rows=[]
def endpoints(d,eye,ids):
 y=ids//640;x=ids%640;r=np.c_[(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(ids))];T=d[eye+'_camera_usd_world'];z=d[eye+'_depth_m'][y,x]
 with np.errstate(invalid='ignore'):p=T[:3,3]+(r*z[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 return p

def cylinder_proxy(p):
 distance=np.linalg.norm(p[:,:2,None]-centres[:,:2].T[None],axis=1);idx=np.argmin(np.where(np.isfinite(distance),distance,np.inf),axis=1);valid=np.isfinite(p).all(1)&(distance[np.arange(len(p)),idx]<=.355)&(p[:,2]>=-.005)&(p[:,2]<=1.005)
 return idx,valid
for method,i,p in groups:
 s=np.load(p);d=np.load(o/'offline_truth'/f'frame_{i:03d}.npz');rgb=np.load(o/'online_inputs'/f'frame_{i:03d}.npz');q=s['query_pixel_id'].astype(int);v=s['partner_pixel_id'].astype(int);a=endpoints(d,'left',q);b=endpoints(d,'right',v);ia,va=cylinder_proxy(a);ib,vb=cylinder_proxy(b);both=va&vb;gap=np.linalg.norm(a-b,axis=1);n=len(q)
 phot=np.mean(np.abs(rgb['left_rgb'].reshape(-1,3)[q].astype(float)-rgb['right_rgb'].reshape(-1,3)[v].astype(float)),axis=1);rows.append(dict(method=method,frame=i,queries=n,left_divider_proxy=int(va.sum()),right_divider_proxy=int(vb.sum()),both_divider_proxy=int(both.sum()),different_nearest_cylinder_proxy=int((both&(ia!=ib)).sum()),same_nearest_cylinder_proxy=int((both&(ia==ib)).sum()),true_endpoint_gap_p50_p90=np.percentile(gap[np.isfinite(gap)],[50,90]).tolist() if np.isfinite(gap).any() else [],selected_RGB_absolute_gap_p50_p90=np.percentile(phot,[50,90]).tolist() if n else [],GT_filtered_candidates_not_used=True))
result=dict(protocol=seal,rows=rows,does_not_identify_the_only_root_cause=True,old_source_rejections_preserved=True,whole_research_complete=False);(H/'retreat_correspondence_alias_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(rows))
