from pathlib import Path
import sys,json,hashlib,time,numpy as np
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parent/'dual_monocular_semantic_bev_20261015_v1'));from common import GROUPS
from projection import per_eye,cells,AX,F
old=json.loads((D.parent/'front_rear_camera_geometry_20261015_v1/coverage_upper_bound.json').read_text());res={};inputs={};begin=time.monotonic()
for g,(R,cp) in GROUPS.items():
 masks=dict(np.load(json.loads(cp.read_text())['static_masks']));origin=np.load(R/'maps/frame_000.npz')['origin'];tot=np.zeros(4,int);rows=[]
 for j,fp in enumerate(sorted((R/'frames').glob('frame_*.npz'))):
  a=np.load(fp);T=a['left_optical_to_base'].copy();T[:3,:3]=T[:3,:3]@AX;B=a['left_camera_usd_world']@np.linalg.inv(T);w=np.concatenate([per_eye(a[e+'_depth_m'],a[e+'_camera_usd_world'],masks[e],a['root_position_world_m'],stride=1)[0] for e in ['left','right']]);b=(w-B[:3,3])@B[:3,:3];candidates=[np.zeros(len(w),bool) for unused in range(3)]
  for rear in [False,True]:
   z=(-b[:,0] if rear else b[:,0])-.45;u=b[:,2]-.05;v=(b[:,1] if rear else -b[:,1]);Z=(z>=.5)&(z<=3);candidates[0]|=Z
   for kk,focal in enumerate([F,F/3]):
    keep=Z&(u>=(-.5-319.5)*z/focal)&(u<(639.5-319.5)*z/focal)&(v>=(-.5-239.5)*z/focal)&(v<(479.5-239.5)*z/focal);candidates[kk+1]|=keep
  cc=[len(cells(w,origin))]+[len(cells(w[k],origin)) for k in candidates];assert cc[2]==old['groups'][g]['frames'][j]['retained_cells_upper_bound'];assert cc[0]==old['groups'][g]['frames'][j]['target_cells'];assert cc[2]<=cc[3]<=cc[1]<=cc[0];tot+=cc;rows.append(dict(frame=fp.name,target=cc[0],range_only_upper_bound=cc[1],original_FOV=cc[2],fixed_wide_FOV=cc[3],range_lost=cc[0]-cc[1],wide_angular_lost=cc[1]-cc[3]));inputs[str(fp)]=hashlib.sha256(fp.read_bytes()).hexdigest()
 res[g]=dict(target_instances=int(tot[0]),range_only_instances=int(tot[1]),narrow_instances=int(tot[2]),wide_instances=int(tot[3]),range_lost=int(tot[0]-tot[1]),wide_angular_lost=int(tot[1]-tot[3]),wide_retention=float(tot[3]/tot[0]),range_retention=float(tot[1]/tot[0]),pass_gate=bool(tot[3]/tot[0]>=.95),frames=rows);print(g,res[g]['range_retention'],res[g]['wide_retention'],flush=True)
(D/'decomposition.json').write_text(json.dumps(dict(groups=res,pass_gate=all(g['pass_gate'] for g in res.values()),focal_px=F/3,pixel_width=640,pixel_height=480,scope='Complete seven seen development GT-only geometric domain factors; no new exposure or neural/source/navigation result, no GT scale-fitting.',wall_s=time.monotonic()-begin,input_sha256=inputs),indent=2)+'\n')
