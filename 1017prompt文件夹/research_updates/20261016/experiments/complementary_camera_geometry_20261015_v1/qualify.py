from pathlib import Path
import sys,json,hashlib,time
import numpy as np
from scipy.spatial.transform import Rotation
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D.parent/'dual_monocular_semantic_bev_20261015_v1'))
from common import GROUPS
from projection import per_eye,cells,AX,F
cfg=json.loads((D/'protocol.json').read_text());Y=[Rotation.from_euler('z',v,degrees=True).as_matrix() for v in cfg['layout']['yaw_degrees']]
def access(w,Ts):
 arms=[]
 for T in Ts:
  c=(w-T[:3,3])@(T[:3,:3]@AX); z=c[:,2];u=F*c[:,0]/np.maximum(z,1e-9)+319.5;v=F*c[:,1]/np.maximum(z,1e-9)+239.5
  arms.append((z>=.5)&(z<=3)&(u>=-.5)&(u<639.5)&(v>=-.5)&(v<479.5))
 return arms
results={};inputs={};start=time.monotonic()
for name,(R,cp) in GROUPS.items():
 conf=json.loads(cp.read_text());maskpath=Path(conf['static_masks']);masks=dict(np.load(maskpath));origin=np.load(R/'maps/frame_000.npz')['origin'];rows=[];tot=[0,0];rigs=[]
 for p in sorted((R/'frames').glob('frame_*.npz')):
  a=np.load(p);root=a['root_position_world_m'];rr=Rotation.from_quat(np.r_[a['root_quaternion_world_wxyz'][1:],a['root_quaternion_world_wxyz'][0]]).as_matrix();Ts=[];ws=[]
  for j,e in enumerate(['left','right']):
   old=a[e+'_camera_usd_world'];pos=(old[:3,3]-root)@rr;assert np.allclose(pos,cfg['layout']['positions_m'][j],atol=2e-5),(name,p,pos)
   new=old.copy();new[:3,:3]=rr@Y[j]@rr.T@old[:3,:3];Ts.append(new);w,_=per_eye(a[e+'_depth_m'],old,masks[e],root,stride=1);ws.append(w)
  w=np.concatenate(ws);target=cells(w,origin);arms=access(w,Ts);visible=cells(w[arms[0]|arms[1]],origin);keep=len(set(map(tuple,target))&set(map(tuple,visible)));tot[0]+=len(target);tot[1]+=keep
  rows.append(dict(frame=p.name,time_s=float(a['time_s']),target_cells=len(target),retained_cells_upper_bound=keep,retention=keep/max(1,len(target))))
  inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
  if name=='B65' and p.name=='frame_470.npz':np.savez_compressed(D/'example.npz',left_rgb=a['left_rgb'],right_rgb=a['right_rgb'],left_gt=a['left_depth_m'],right_gt=a['right_depth_m'],target_cells=target,retained_cells=visible,origin=origin,root=root)
 results[name]=dict(target_cell_instances=tot[0],retained_cell_instances_upper_bound=tot[1],retention_upper_bound=tot[1]/max(1,tot[0]),frames=rows,pass_gate=tot[1]/max(1,tot[0])>=.95);print(name,results[name]['retention_upper_bound'],flush=True)
angles=np.linspace(-np.pi,np.pi,3600,endpoint=False);ring=[]
ROT=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
for r in [.55,1.,2.,3.]:
 w=np.column_stack([r*np.cos(angles),r*np.sin(angles),np.zeros(len(angles))]);old=[];new=[]
 for j,pos in enumerate(cfg['layout']['positions_m']):
  T=np.eye(4);T[:3,3]=pos;T[:3,:3]=ROT@AX;old.append(T);T=T.copy();T[:3,:3]=Y[j]@T[:3,:3];new.append(T)
 b=access(w,old);c=access(w,new);bu=b[0]|b[1];cu=c[0]|c[1];ring.append(dict(radius_m=r,angles=3600,baseline_visible=int(bu.sum()),candidate_visible=int(cu.sum()),baseline_retained=int((bu&cu).sum()),front_center_candidate=bool(access(np.array([[r,0,0]]),new)[0][0] or access(np.array([[r,0,0]]),new)[1][0])))
 np.savez_compressed(D/f'ring_{r}.npz',angle=angles,baseline=bu,candidate=cu)
res=dict(groups=results,ring=ring,pass_gate=all(v['pass_gate'] for v in results.values()),physical=0,fit=0,scope='Calibrated no-added-occlusion geometric upper bound. Not rendered new-view RGB/depth, actual union scene recall, model performance or navigation.',wall_s=time.monotonic()-start)
(D/'results.json').write_text(json.dumps(res,indent=2)+'\n');(D/'input_seal.json').write_text(json.dumps(inputs,indent=2)+'\n')
