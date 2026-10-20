from pathlib import Path
import sys,json,time,hashlib,numpy as np
W=Path.cwd();D=W/'experiments/camera_occlusion_identity_20261015_v1';R=W/'experiments/original_four_camera_20261015_v1';sys.path.insert(0,str(R))
from cad_masks import FourCAD,mask_triangles
from four_camera import nominal_transforms,NAMES
c=FourCAD();rows=[];start=time.time();F=584.8888888888889
# Expanded image edges used by original rasterizer, conservative by >1 pixel.
planes=np.array([[0,0,1],[1,0,320.5/F],[-1,0,320.5/F],[0,1,240.5/F],[0,-1,240.5/F]])
for p in sorted((R/'native_run/frames').glob('frame_*.npz')):
 a=np.load(p);b=np.load(R/(p.stem+'_audit.npz'));T=c.cad.body_transforms(a['joints_current']);tri=np.concatenate([v@T[k][:3,:3].T+T[k][:3,3] for k,v in c.cad.triangles.items()]);arms=[]
 for i,name in enumerate(NAMES):
  out={}
  for tag,x,z in [('saved020005',.2,.05),('user024008',.24,.08)]:
   cam=nominal_transforms(x,.3,z)[i];o=(tri-cam[:3,3])@(cam[:3,:3]@np.diag([1,-1,-1]));m=mask_triangles(o,640,480,F,319.5,239.5)
   outside=np.zeros(len(o),bool)
   for j,n in enumerate(planes):outside|=((o@n-(.01 if j==0 else 0))<0).all(axis=1)
   out[tag]={'mask_pixels':int(m.sum()),'triangles_not_proven_outside_conservative_frustum':int((~outside).sum())}
   if tag=='saved020005':assert np.array_equal(m,b[name+'_CAD'])
  arms.append(out)
 rows.append({'frame':p.name,'eyes':arms});print(p.name,arms,flush=True)
oldpath=W/'experiments/nearfield_rig_navigation_20261012_v1/forward_extrinsics.npz';e=np.load(oldpath);old={k:e[k].tolist() for k in e.files};m=np.load(oldpath.with_name('forward_masks.npz'))
result={'rows':rows,'old_forward_extrinsics':old,'old_saved_mask_counts':{k:int(m[k].sum()) for k in m.files},'old_extrinsics_sha256':hashlib.sha256(oldpath.read_bytes()).hexdigest(),'old32_masks_exact':True,'new32_masks_zero':all(a['user024008']['mask_pixels']==0 for r in rows for a in r['eyes']),'new32_conservative_frustum_certified':all(a['user024008']['triangles_not_proven_outside_conservative_frustum']==0 for r in rows for a in r['eyes']),'wall_s':time.time()-start,'physical':0,'model':0,'scope':'saved 8 posture CAD only, not fresh RGB; all prior candidate budgets unchanged'}
(D/'result.json').write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k not in ['rows','old_forward_extrinsics']},flush=True)
