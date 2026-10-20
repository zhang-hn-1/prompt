from pathlib import Path
import json,sys,itertools,hashlib,time,numpy as np
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parent/'dual_monocular_semantic_bev_20261015_v1'));from common import GROUPS
from dynamic_self_mask import RobotSelfMask
cfg=json.loads((D/'protocol.json').read_text());cad=RobotSelfMask();verts={k:t.reshape(-1,3) for k,t in cad.triangles.items()};bounds={k:np.array(list(itertools.product(*zip(v.min(0),v.max(0))))) for k,v in verts.items()};rows=[];sources={};begin=time.monotonic()
# If every triangle vertex is behind z=.01 then no triangle can enter the frustum: exact zero mask, for both eyes.
for group,(R,cp) in GROUPS.items():
 p=R/'physics.npz';a=np.load(p);sources[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest();n=0;fail=[]
 for fp in sorted((R/'frames').glob('frame_*.npz')):
  t=float(np.load(fp)['time_s']);i=int(np.argmin(abs(a['simulation_time_s']-t)));assert abs(float(a['simulation_time_s'][i])-t)<1e-4
  qs=a['joint_position_rad'][i];T=cad.body_transforms(qs);front=-np.inf;rear=-np.inf;parts=[]
  for k,v in verts.items():
   x=bounds[k]@T[k][:3,0]+T[k][0,3];upper=max(x)-.45;lower=-min(x)-.45
   if max(upper,lower)>=.01:
    x=v@T[k][:3,0]+T[k][0,3];upper=max(x)-.45;lower=-min(x)-.45
   front=max(front,upper);rear=max(rear,lower);parts.append([k,float(upper),float(lower)])
  if max(front,rear)>=.01:fail.append(fp.name)
  rows.append(dict(group=group,frame=fp.name,source_time_s=t,physics_index=i,joints=qs.tolist(),max_front_optical_z=float(front),max_rear_optical_z=float(rear),zero_CAD_certified=bool(max(front,rear)<.01)));n+=1
 assert n==181;print(group,n,'uncertified',len(fail),flush=True)
res=dict(complete_frames=len(rows),rows=rows,pass_gate=all(x['zero_CAD_certified'] for x in rows),scope='Known CAD actual recorded three-joint poses, exact all-vertex depth bound after conservative box broad phase; no scene pixels, GT depth or model. Does not certify scene visibility/semantic/source/navigation. Upper bound cannot certify zero if any vertex forward; no fallback threshold.',wall_s=time.monotonic()-begin,CAD_source_identity=cad.identity,physics_sha256=sources)
(D/'cad_qualification.json').write_text(json.dumps(res,indent=2)+'\n');np.savez_compressed(D/'zero_masks.npz',front=np.zeros((480,640),bool),rear=np.zeros((480,640),bool))
