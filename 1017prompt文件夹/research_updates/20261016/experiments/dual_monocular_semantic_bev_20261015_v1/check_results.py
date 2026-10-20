from common import *
from category_memory import CategoryMemory
import numpy as np
from scipy.spatial import cKDTree
r=json.loads((D/'result.json').read_text());seal=json.loads((D/'prediction_seal.json').read_text());n=0;mapn=0;memcomparisons={}
for name,g in r['groups'].items():
 for j,a in enumerate(g['arms']):
  v=np.array([x['bev_counts'][j] for x in g['frames']]).sum(0);assert list(v)==[a['predicted'],a['true_union'],a['correct'],a['covered']];assert abs(v[2]/max(1,v[0])-a['precision'])<1e-14;assert abs(v[3]/max(1,v[1])-a['recall'])<1e-14;n+=len(g['frames'])
 for j,a in enumerate(g['depths']):
  fs=[x['eyes'][j] for x in g['frames']];assert sum(x['valid_true_pixels'] for x in fs)==a['true_domain'];assert sum(x['ground_obstacle_false'] for x in fs)==a['ground_false'];assert sum(x['true_ground_pixels'] for x in fs)==a['ground_denominator']
 R,cp=GROUPS[name];cfg=json.loads(cp.read_text());init=np.load(R/'maps/frame_000.npz');m=CategoryMemory(init['origin'],init['labels'].shape,.1,name!='H');gm=D/'predictions'/name;preds={int(p.stem[6:]):p for p in gm.glob('frame_*.npz')};expected=next(x for x in seal['rows'] if x['group']==name)['memory_counts'];occ=np.load(cfg['occupancy']);tree=cKDTree(occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m']));comparison=[]
 for i in range(900):
  if i%5==0:
   a=np.load(preds[i]);pose=np.load(R/'known_poses'/preds[i].name);c=a['fused_cells'];w=np.column_stack([init['origin']+c*.1,np.full(len(c),.5)]);m.update(w,np.full(len(c),2),i,float(a['time_s']),pose['left_camera_usd_world'][:3,3])
  original=np.load(R/'maps'/f'frame_{i:03d}.npz');t=float(original['time_s']);s=m.snapshot(t);assert int((s['labels']==1).sum())==expected[i]['risk'];mapn+=1
  if i in [450,495,500,899]:
   saved=np.load(gm/f'memory_{i:03d}.npz');assert np.array_equal(s['labels'],saved['labels']);assert np.array_equal(s['clock'],saved['clock']);ids=np.argwhere(original['labels']==1);xy=original['origin']+ids*.1;dist=tree.query(xy)[0] if len(ids) else np.empty(0);old=t-original['hazard_acquisition_time'][tuple(ids.T)]>10;comparison.append(dict(frame=i,original_risk=len(ids),original_wrong=int((dist>.15).sum()),original_old_wrong=int(((dist>.15)&old).sum()),candidate=next(x for x in g['memory'] if x['frame']==i)))
 memcomparisons[name]=comparison
# independently recompute depth quantiles for 14 fixed mid-sequence eye frames, no model rerun.
checks=[]
for name,(R,cp) in GROUPS.items():
 cfg=json.loads(cp.read_text());masks=np.load(cfg['static_masks']);p='frame_450.npz';f=np.load(R/'frames'/p);a=np.load(D/'predictions'/name/p);fr=next(x for x in r['groups'][name]['frames'] if x['frame']==p)
 for j,e in enumerate(['left','right']):
  gt=f[e+'_depth_m'];z=a[e+'_depth_m'];valid=np.isfinite(gt)&(gt>=.5)&(gt<=3)&~masks[e].astype(bool)&np.isfinite(z);p90=float(np.quantile(np.abs(gt[valid]-z[valid]),.9));assert abs(p90-fr['eyes'][j]['depth_p90_m'])<1e-12;checks.append([name,e,p90])
save('independent_result_check.json',dict(pass_check=True,bev_arm_frames=n,memory_ticks=mapn,depth_quantile_checks=checks,memory_comparisons=memcomparisons,scope='Independent full count aggregation; complete same implementation memory replay exact 6300 counts/28 labels+clock snapshots; independent14 current-depth quantiles. Not second model inference or all BEV geometry independent implementation.'))
print('FULL_CHECK_PASS',n,mapn,flush=True)
