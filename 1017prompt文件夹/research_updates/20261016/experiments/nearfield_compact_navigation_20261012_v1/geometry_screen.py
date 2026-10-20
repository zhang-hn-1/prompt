import json,sys,itertools
from pathlib import Path
import numpy as np
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path.insert(0,str(W/'experiments/stereo_memory_navigation_20261004_v1'))
from rig_adapter import Rig,RigMask,ROTATION,FOCAL
p=json.loads((D/'protocol.json').read_text());s=json.loads((W/'experiments/visual_imu_navigation90_20261012_v1/e2_run/states.json').read_text())
joints=[np.zeros(3)]+[np.asarray(s[i]['joints']) for i in [100,150,180,190,195]]
grid=p['target_grid'];records=[]
for item in p['offline_candidates']:
 rig=Rig(**item);masker=RigMask(rig);rows=[]
 for q in joints:
  masks=masker.project(q);r={}
  for x in grid['body_x_m']:
   targets=np.array(list(itertools.product([x],grid['body_y_m'],grid['body_z_m'])))
   support=np.ones(len(targets),bool)
   for eye in ['left','right']:
    optical=(targets-rig.translation(eye))@ROTATION;depth=optical[:,2]
    uv=optical[:,:2]/depth[:,None]*FOCAL+[319.5,239.5]
    inside=(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479)&(depth>=.5)&(depth<=3)&(FOCAL*rig.baseline/depth<448)
    u=np.clip(np.rint(uv[:,0]),0,639).astype(int);v=np.clip(np.rint(uv[:,1]),0,479).astype(int)
    support &= inside & ~masks[eye][v,u]
   r[str(x)]=int(support.sum())
  rows.append(r)
 z=3.;disparity=FOCAL*rig.baseline/z;uncertainty=FOCAL*rig.baseline/(disparity-1)-z
 near=min(sum(row[str(x)] for x in grid['body_x_m'] if x<=1.5) for row in rows);far=min(sum(row[str(x)] for x in grid['body_x_m'] if x>=2) for row in rows)
 records.append(dict(rig=item,joint_rows=rows,near_worst=near,far_worst=far,depth3m_plus1px_uncertainty_m=uncertainty))
 np.savez_compressed(D/(rig.name+'_masks.npz'),**masker.frozen)
 np.savez_compressed(D/(rig.name+'_extrinsics.npz'),**{eye+'_optical_to_base':rig.optical(eye) for eye in ['left','right']})
base=records[0]
for r in records:r['geometry_pass']=r['near_worst']>base['near_worst'] and r['far_worst']>=.8*base['far_worst'] and r['depth3m_plus1px_uncertainty_m']<.1
selected=next((r['rig'] for r in records[1:] if r['geometry_pass']),None)
out=dict(records=records,selected=selected,scope='CAD sampled targets and first-order disparity uncertainty only; no real obstacle visibility, scene stereo accuracy or navigation claim')
(D/'geometry_assessment.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
