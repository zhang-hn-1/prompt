from pathlib import Path
import sys,json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
D=Path(__file__).resolve().parent;sys.path.insert(0,str(D.parent/'dual_monocular_semantic_bev_20261015_v1'));from common import GROUPS
from projection import per_eye,cells,AX,F
cfg=json.loads((D/'protocol.json').read_text());ROT=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
def local(pos,yaw):
 T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@ROT@AX;T[:3,3]=pos;return T
CT=[local(cfg['layout']['front_position_m'],0),local(cfg['layout']['rear_position_m'],180)]
rows={};inputs={}
for g,(R,cp) in GROUPS.items():
 masks=dict(np.load(json.loads(cp.read_text())['static_masks']));origin=np.load(R/'maps/frame_000.npz')['origin'];rr=[];tot=[0,0];parentdiff=[]
 for fp in sorted((R/'frames').glob('frame_*.npz')):
  a=np.load(fp);body=[];ws=[]
  for e in ['left','right']:
   T=a[e+'_optical_to_base'].copy();T[:3,:3]=T[:3,:3]@AX;body.append(a[e+'_camera_usd_world']@np.linalg.inv(T));w,_=per_eye(a[e+'_depth_m'],a[e+'_camera_usd_world'],masks[e],a['root_position_world_m'],stride=1);ws.append(w)
  delta=np.max(np.abs(body[0]-body[1]));assert delta<2e-5,(g,fp,delta);parentdiff.append(float(delta));Ts=[body[0]@x for x in CT];w=np.concatenate(ws);target=cells(w,origin);valid=[]
  for T in Ts:
   c=(w-T[:3,3])@(T[:3,:3]@AX);z=c[:,2];u=F*c[:,0]/np.maximum(z,1e-9)+319.5;v=F*c[:,1]/np.maximum(z,1e-9)+239.5;valid.append((z>=.5)&(z<=3)&(u>=-.5)&(u<639.5)&(v>=-.5)&(v<479.5))
  vc=cells(w[valid[0]|valid[1]],origin);ret=len(set(map(tuple,target))&set(map(tuple,vc)));tot[0]+=len(target);tot[1]+=ret;rr.append(dict(frame=fp.name,target_cells=len(target),retained_cells_upper_bound=ret,source_time_s=float(a['time_s']),GT_parent_agreement=delta));inputs[str(fp)]=hashlib.sha256(fp.read_bytes()).hexdigest()
  if g=='B65' and fp.name=='frame_000.npz':np.savez_compressed(D/'coverage_example.npz',origin=origin,target_cells=target,retained_cells=vc)
 rows[g]=dict(target_instances=tot[0],retained_instances=tot[1],retention_upper_bound=tot[1]/max(1,tot[0]),pass_gate=tot[1]/max(1,tot[0])>=.95,max_GT_parent_eye_disagreement=max(parentdiff),frames=rr);print(g,rows[g]['retention_upper_bound'],flush=True)
(D/'coverage_upper_bound.json').write_text(json.dumps(dict(groups=rows,pass_gate=all(r['pass_gate'] for r in rows.values()),scope='Complete original-domain scene points reprojected with renderer GT base-link transform derived independently from both saved camera calibration/readback, not exact native physics articulation root. No new-view scene/CAD occlusion; upper bound only, no new union recall. Completes frozen original .95 retention gate even when rendering replay interface unavailable.',input_sha256=inputs,physical=0,fit=0),indent=2)+'\n')
