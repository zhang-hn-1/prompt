"""Truth-only complete evaluation, requires immutable prediction seal first."""
from common import *
from projection import *
from scipy.spatial import cKDTree
seal=json.loads((D/'prediction_seal.json').read_text());assert not seal['truth_read']
for part in ['inputs','outputs','sources']:
 for p,h in seal[part].items():assert sha(p)==h,p
results={};truth_hashes={}
for name,(R,cp) in GROUPS.items():
 cfg=json.loads(cp.read_text());masks=dict(np.load(cfg['static_masks']));origin=np.load(R/'maps/frame_000.npz')['origin'];rows=[];totals=np.zeros((3,4),int);errors=[[],[],[]];de=[[],[]];depth_counts=np.zeros((2,3),int);ground_counts=np.zeros((2,2),int);catastrophic=[]
 for p in sorted((R/'source_inputs').glob('*.npz')):
  gtpath=R/'frames'/p.name;f=np.load(gtpath);truth_hashes[str(gtpath)]=sha(gtpath);a=np.load(D/'predictions'/name/p.name);pose=np.load(R/'known_poses'/p.name);target=[];one=[]
  for j,e in enumerate(['left','right']):
   gt=f[e+'_depth_m'];z=a[e+'_depth_m'];valid=np.isfinite(gt)&(gt>=.5)&(gt<=3)&~masks[e].astype(bool);finite=valid&np.isfinite(z);err=np.abs(z[finite]-gt[finite]);de[j].extend(err);depth_counts[j]+=[int(valid.sum()),int(finite.sum()),int((err>.3).sum())];p90=float(np.quantile(err,.9)) if len(err) else None
   if len(err)>=1000 and p90>.3:catastrophic.append([p.name,e])
   w,q=world_points(gt,f[e+'_camera_usd_world'],masks[e],pose['root_position_world_m'],stride=1);hazard=(w[:,2]>.05)&(w[:,2]<=1.5);floor=np.abs(w[:,2])<=.03;cl=a[e+'_category'].reshape(-1)[q];ground_counts[j]+=[int(((cl==1)&floor).sum()),int(floor.sum())];target.append(cells(w[hazard],origin));one.append(dict(valid_true_pixels=int(valid.sum()),finite_prediction_pixels=int(finite.sum()),depth_p90_m=p90,above030=int((err>.3).sum()),ground_obstacle_false=int(((cl==1)&floor).sum()),true_ground_pixels=int(floor.sum()),semantic_unknown_pixels=int((cl==0).sum()),semantic_obstacle_pixels=int((cl==1).sum())))
  target=np.unique(np.concatenate(target),axis=0);bevs=[]
  for j,key in enumerate(['left_cells','right_cells','fused_cells']):
   c=a[key];pd=cKDTree(target*.1).query(c*.1)[0] if len(target) and len(c) else np.full(len(c),np.inf);td=cKDTree(c*.1).query(target*.1)[0] if len(target) and len(c) else np.full(len(target),np.inf);counts=[len(c),len(target),int((pd<=.15+1e-10).sum()),int((td<=.15+1e-10).sum())];totals[j]+=counts;errors[j].extend(pd[np.isfinite(pd)]);bevs.append(counts)
  rows.append(dict(frame=p.name,eyes=one,bev_counts=bevs))
 arms=[]
 for j,(n,t,tp,covered) in enumerate(totals):arms.append(dict(predicted=int(n),true_union=int(t),correct=int(tp),covered=int(covered),precision=float(tp/max(1,n)),recall=float(covered/max(1,t)),position_p90_m=float(np.quantile(errors[j],.9)) if errors[j] else None))
 depths=[]
 for j in range(2):depths.append(dict(true_domain=int(depth_counts[j,0]),finite_predicted=int(depth_counts[j,1]),coverage=float(depth_counts[j,1]/max(1,depth_counts[j,0])),p50_p90_p95_m=np.quantile(de[j],[.5,.9,.95]).tolist() if de[j] else [None]*3,above030_fraction=float(depth_counts[j,2]/max(1,depth_counts[j,1])),ground_false=int(ground_counts[j,0]),ground_denominator=int(ground_counts[j,1]),ground_false_fraction=float(ground_counts[j,0]/max(1,ground_counts[j,1]))))
 occ=np.load(cfg['occupancy']);truth_hashes[cfg['occupancy']]=sha(Path(cfg['occupancy']));tree=cKDTree(occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']));mem=[]
 for i in [450,495,500,899]:
  m=np.load(D/'predictions'/name/f'memory_{i:03d}.npz');ids=np.argwhere(m['labels']==1);xy=m['origin']+ids*float(m['resolution']);d=tree.query(xy)[0] if len(ids) else np.empty(0);old=float(m['time_s'])-m['clock'][tuple(ids.T)]>10;mem.append(dict(frame=i,risk=len(ids),wrong=int((d>.15).sum()),old=int(old.sum()),old_wrong=int(((d>.15)&old).sum())))
 timing=[x['pair_projection_wall_s'] for x in next(r for r in seal['rows'] if r['group']==name)['timings']];c=arms[2];gates=dict(precision=c['precision']>=.95,recall=c['recall']>=.8,location=c['position_p90_m'] is not None and c['position_p90_m']<=.15,depth=all(d['p50_p90_p95_m'][1] is not None and d['p50_p90_p95_m'][1]<=.15 for d in depths),depth_coverage=all(d['coverage']>=.95 for d in depths),ground_false=all(d['ground_false_fraction']<=.01 for d in depths),catastrophic=not catastrophic,latency=float(np.quantile(timing,.9))<=.5);results[name]=dict(arms=arms,depths=depths,gates=gates,catastrophic_frames=catastrophic,frames=rows,memory=mem,pair_timing_p50_p90_max=np.quantile(timing,[.5,.9,1]).tolist());print(name,gates,flush=True)
save('result.json',dict(groups=results,pass_gate=all(all(g['gates'].values()) for g in results.values()),truth_hashes=truth_hashes,fit=0,physical=0,scope='Full current left/right union visible target, all true finite .5–3m pixels scored even if model depth outside range; no GT rescale. Pixel floor/unknown and cell-instance denominators distinct. Same trajectory offline category memory, not navigation or heldout generalization.'))
print('ALL GROUPS SCORED',flush=True)
