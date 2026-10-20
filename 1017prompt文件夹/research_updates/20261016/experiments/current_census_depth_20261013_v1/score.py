"""Separate GT-only scoring after all current-pair and map predictions are sealed."""
from common import *
import numpy as np
from scipy.spatial import cKDTree

def quant(a):return np.quantile(a,[.5,.9,.95]).tolist() if len(a) else [None]*3

def point_truth(f,q):
 x=q%640;y=q//640;gt=f['left_depth_m'][y,x];finite=np.isfinite(gt)&(gt>0);T=f['left_camera_usd_world'];rays=np.stack([(x-319.5)/584.8888888888889,(y-239.5)/584.8888888888889,np.ones(len(q))],1)
 world=T[:3,3]+(rays*np.where(finite,gt,0)[:,None])@(T[:3,:3]@np.diag([1,-1,-1])).T
 return finite,world

def main():
 seal=json.loads((D/'prediction_seal.json').read_text())
 for section in ['inputs','outputs','sources']:
  for p,s in seal[section].items():assert sha(p)==s,p
 result=[];truth_sha={}
 for name,(R,cp) in GROUPS.items():
  errors=[[],[]];point_errors=[[],[]];counts=np.zeros((2,7),np.int64);rows=[];obtotal=obkept=0;gm=D/'predictions'/name
  for p in sorted((R/'source_predictions').glob('*.npz')):
   i=int(p.stem.split('_')[1]);f=np.load(R/'frames'/p.name);truth_sha[str(R/'frames'/p.name)]=sha(R/'frames'/p.name);gt=f['left_depth_m'];finite=np.isfinite(gt)&(gt>0);fr=dict(frame=i,time_s=float(f['time_s']),arms=[]);surfs=[]
   for j,(a,sp) in enumerate([(np.load(p),R/'partial_observations'/p.name),(np.load(gm/p.name),gm/f'surface_{i:03d}.npz')]):
    z=a['depth_m'];valid=a['confidence'].astype(bool)&np.isfinite(z)&(z>=.5)&(z<=3);k=valid&finite;e=np.abs(z[k]-gt[k]);errors[j].append(e)
    sf=np.load(sp);q=sf['query_pixel_id'];ok,truth=point_truth(f,q);ground=ok&(np.abs(truth[:,2])<=.03);obs=ok&(truth[:,2]>.05)&(truth[:,2]<=1.5)
    pe=np.linalg.norm(sf['world'][ok]-truth[ok],axis=1);point_errors[j].append(pe);surfs.append((q,obs))
    counts[j]+=np.array([valid.sum(),k.sum(),(valid&np.isposinf(gt)).sum(),len(q),ok.sum(),ground.sum(),obs.sum()])
    fr['arms'].append(dict(predicted=int(valid.sum()),finite=int(k.sum()),p50_p90_p95=quant(e),above030_fraction=float((e>.3).mean()) if len(e) else None,points=len(q),finite_points=int(ok.sum()),ground_false=int(ground.sum()),point_p50_p90_p95=quant(pe)))
   obq=surfs[0][0][surfs[0][1]];obtotal+=len(obq);obkept+=int(np.isin(obq,surfs[1][0]).sum());rows.append(fr)
  arms=[]
  for j,title in enumerate(['baseline','census']):
   e=np.concatenate(errors[j]);pe=np.concatenate(point_errors[j]);n,nf,ni,npnt,npf,ng,no=map(int,counts[j]);arms.append(dict(arm=title,predicted=n,finite=nf,finite_scoring_coverage=nf/max(1,n),full_image_scored_fraction=nf/(181*480*640),nohit=ni,depth_p50_p90_p95=quant(e),above030_fraction=float((e>.3).mean()) if len(e) else None,accepted_points=npnt,finite_accepted_points=npf,ground_false_points=ng,ground_false_fraction=ng/max(1,npf),gt_obstacle_points=no,point_error_p50_p90_p95=quant(pe)))
  cand=arms[1];tail=[r['frame'] for r in rows if r['arms'][1]['finite']>=1000 and r['arms'][1]['p50_p90_p95'][1]>.3];coveragefail=[r['frame'] for r in rows if r['arms'][0]['predicted']>=1000 and r['arms'][1]['predicted']<.5*r['arms'][0]['predicted']]
  gates=dict(finite_count=cand['finite']>=1000,depth_p90=cand['depth_p50_p90_p95'][1] is not None and cand['depth_p50_p90_p95'][1]<=.15,outlier=cand['above030_fraction'] is not None and cand['above030_fraction']<=.1,dense_coverage=cand['predicted']>=.7*arms[0]['predicted'],per_frame_coverage=not coveragefail,catastrophic_frame=not tail,obstacle_retention=obtotal>0 and obkept>=.95*obtotal)
  cfg=json.loads(cp.read_text());occ=np.load(cfg['occupancy']);truth_sha[cfg['occupancy']]=sha(cfg['occupancy']);xy=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']);tree=cKDTree(xy);maps=[]
  for i in [450,495,500,899]:
   pair=[]
   for title in ['baseline','candidate']:
    m=np.load(gm/f'map_{title}_{i:03d}.npz');cells=np.argwhere(m['labels']==1);p=m['origin']+cells*float(m['resolution']);old=float(m['time_s'])-m['clock'][tuple(cells.T)]>10;dist=tree.query(p)[0];bad=dist>.15
    pair.append(dict(arm=title,risk_cells=len(cells),far015=int(bad.sum()),older10s=int(old.sum()),old_far015=int((old&bad).sum()),cell_distance_p50_p90_p95=quant(dist)))
   maps.append(dict(frame=i,time_s=float(m['time_s']),arms=pair))
  r=dict(group=name,arms=arms,original_true_obstacle_queries=obtotal,still_measured_obstacle_queries=obkept,obstacle_query_retention=obkept/max(1,obtotal),dense_count_ratio=cand['predicted']/max(1,arms[0]['predicted']),catastrophic_frames=tail,frame_coverage_failures=coveragefail,gates=gates,source_gate=all(gates.values()),frames=rows,maps=maps);result.append(r);print(name, {k:v for k,v in r.items() if k not in ['frames','maps']},flush=True)
 save('score_truth_sha.json',truth_sha);save('result.json',dict(groups=result,source_pass=all(r['source_gate'] for r in result),physical_used=0,fit=0,scope='five complete seen development groups; no new navigation; generated maps before GT at original trajectory'))
 ledger=json.loads((D/'ledger.json').read_text());passed=all(r['source_gate'] for r in result);ledger.update(status='source_qualified_requires_navigation' if passed else 'source_rejected',physical_unattempted_closed=0 if passed else 4);save('ledger.json',ledger)
if __name__=='__main__':main()
