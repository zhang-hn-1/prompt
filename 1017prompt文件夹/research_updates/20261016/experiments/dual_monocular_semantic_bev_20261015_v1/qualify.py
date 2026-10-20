"""Explicit GT-only oracle geometry qualification before model execution."""
from common import *
from projection import *
from semantic_projection import project_semantic
from scipy.spatial import cKDTree
results={}
for name,(R,cp) in GROUPS.items():
 cfg=json.loads(cp.read_text());masks=dict(np.load(cfg['static_masks']));origin=np.load(R/'maps/frame_000.npz')['origin'];rows=[];totals=np.zeros((3,4),int);errs=[[],[],[]];angles=[]
 for p in sorted((R/'source_inputs').glob('*.npz')):
  a=np.load(p);pose=dict(np.load(R/'known_poses'/p.name));f=np.load(R/'frames'/p.name);zs=[f[e+'_depth_m'] for e in ['left','right']];classes=[]
  for j,e in enumerate(['left','right']):
   w,q=world_points(zs[j],f[e+'_camera_usd_world'],masks[e],pose['root_position_world_m'],stride=1);cl=np.zeros((480,640),np.uint8);cl.reshape(-1)[q[(w[:,2]>.05)&(w[:,2]<=1.5)]]=1;classes.append(cl)
  pred,conflict=project_semantic(zs,classes,pose,masks,origin);targets=[]
  for j,e in enumerate(['left','right']):
   w,_=per_eye(zs[j],f[e+'_camera_usd_world'],masks[e],pose['root_position_world_m'],stride=1);targets.append(cells(w,origin))
  target=np.unique(np.concatenate(targets),axis=0);one=[]
  for j,c in enumerate(pred):
   pd=cKDTree(target*.1).query(c*.1)[0] if len(target) and len(c) else np.full(len(c),np.inf);td=cKDTree(c*.1).query(target*.1)[0] if len(target) and len(c) else np.full(len(target),np.inf);counts=[len(c),len(target),int((pd<=.15+1e-10).sum()),int((td<=.15+1e-10).sum())];totals[j]+=counts;errs[j].extend(pd[np.isfinite(pd)]);one.append(counts)
  L=pose['left_camera_usd_world'][:3,:3];Rt=pose['right_camera_usd_world'][:3,:3];angles.append(float(np.arccos(np.clip((np.trace(L.T@Rt)-1)/2,-1,1))));rows.append(dict(frame=p.name,counts=one,conflicts=conflict));assert a['left_rgb'].shape==a['right_rgb'].shape==(480,640,3)
 arms=[]
 for j,(n,t,tp,covered) in enumerate(totals):arms.append(dict(predicted=int(n),union_target=int(t),precision=float(tp/max(1,n)),recall=float(covered/max(1,t)),location_p90=float(np.quantile(errs[j],.9)) if errs[j] else None))
 c=arms[2];gates=dict(precision=c['precision']>=.95,recall=c['recall']>=.8,location=c['location_p90'] is not None and c['location_p90']<=.15);results[name]=dict(arms=arms,gates=gates,frames=rows,max_camera_axis_angle_rad=max(angles));print(name,arms[2],gates,flush=True)
save('oracle_qualification.json',dict(groups=results,pass_gate=all(all(r['gates'].values()) for r in results.values()),scope='GT-only geometric probe, never online. Fixed full union targets with explicit CAD/range/self domain, same target denominator for each eye and fusion. Native estimated camera pose vs renderer pose; old ideal IMU/initial convention disclosed. Not neural model performance.'))
