"""Bounded, post-seal GT diagnostic of the prior 46-pixel blocker cohort only."""
from common import *
import numpy as np,cv2
from rig_adapter import Rig,RigMask
from dynamic_self_mask import joints_from_proprio
from census import census_codes
assert (D/'prediction_seal.json').exists();R,_=GROUPS['C54'];cad=RigMask(Rig('forward',.4,.3));prior=json.loads((O/'blocker_provenance.json').read_text());rows=[];input_sha={}
focal=584.8888888888889
for item in prior['source_epochs']:
 i=int(item['frame'].split('_')[1]);p=R/'frames'/f'frame_{i:03d}.npz';f=np.load(p);histpath=R/'history'/p.name;hist=np.load(histpath)['proprio_history'];masks=cad.project(joints_from_proprio(hist));q=np.array(item['pixel_ids']);x=q%640;y=q//640;z=f['left_depth_m'][y,x];TL=f['left_camera_usd_world'];TR=f['right_camera_usd_world'];rot=np.diag([1,-1,-1]);world=TL[:3,3]+(np.stack([(x-319.5)/focal,(y-239.5)/focal,np.ones(len(q))],1)*z[:,None])@(TL[:3,:3]@rot).T;right=(world-TR[:3,3])@(TR[:3,:3]@rot);uv=np.column_stack([right[:,0]/right[:,2]*focal+319.5,right[:,1]/right[:,2]*focal+239.5]);idx=np.rint(uv).astype(int);inside=(idx[:,0]>=0)&(idx[:,0]<640)&(idx[:,1]>=0)&(idx[:,1]<480)&(right[:,2]>0);rx=np.clip(idx[:,0],0,639);ry=np.clip(idx[:,1],0,479);rz=f['right_depth_m'][ry,rx];good=np.isfinite(rz)&(rz>0);gap=rz-right[:,2]
 lm=~cv2.dilate(masks['left'].astype(np.uint8),np.ones((9,9),np.uint8)).astype(bool);rm=~cv2.dilate(masks['right'].astype(np.uint8),np.ones((9,9),np.uint8)).astype(bool)
 old=np.load(R/'source_predictions'/p.name);new=np.load(D/'predictions/C54'/p.name);L=census_codes(cv2.cvtColor(f['left_rgb'],cv2.COLOR_RGB2GRAY));RR=census_codes(cv2.cvtColor(f['right_rgb'],cv2.COLOR_RGB2GRAY))
 def cost(cx,cy,px,py):
  if not (4<=cx<636 and 4<=px<636 and 4<=cy<476 and 4<=py<476):return None
  return sum(int(int(L[cy+dy,cx+dx])^int(RR[py+dy,px+dx])).bit_count() for dy in range(-1,2) for dx in range(-1,2))
 # Native-window bit order differs by a fixed permutation from rectified; Hamming remains invariant.
 for n,pid in enumerate(q):
  predz=float(new['depth_m'].ravel()[pid]);d=float(np.rot90(new['disparity_px'],-1).ravel()[pid]);wrongpy=int(round(y[n]-d));row=dict(frame=i,pixel=int(pid),true_depth_m=float(z[n]),true_partner_uv=uv[n].tolist(),inside=bool(inside[n]),right_GT_finite=bool(good[n]),right_depth_gap_m=float(gap[n]) if good[n] else None,right_depth_compatible_with015m=bool(inside[n]&good[n]&(abs(gap[n])<=.15)),left_9x9_current_CAD_clear=bool(lm[y[n],x[n]]),right_9x9_current_CAD_clear=bool(rm[ry[n],rx[n]]),true_integer_cost=cost(x[n],y[n],rx[n],ry[n]),chosen_integer_cost=cost(x[n],y[n],x[n],wrongpy),new_depth_m=predz if np.isfinite(predz) else None);rows.append(row)
 for fpath in [p,histpath,R/'source_predictions'/p.name,D/'predictions/C54'/p.name]:input_sha[str(fpath)]=sha(fpath)
summary=dict(total=len(rows),inside=sum(r['inside'] for r in rows),GT_right_compatible=sum(r['right_depth_compatible_with015m'] for r in rows),left_current_CAD_clear=sum(r['left_9x9_current_CAD_clear'] for r in rows),right_current_CAD_clear=sum(r['right_9x9_current_CAD_clear'] for r in rows),true_cost_above_chosen=sum(r['true_integer_cost'] is not None and r['chosen_integer_cost'] is not None and r['true_integer_cost']>r['chosen_integer_cost'] for r in rows))
save('blocker_visibility_diagnostic.json',dict(summary=summary,rows=rows,input_sha=input_sha,scope='46 previously chosen prior-failure queries only; posthoc GT-right nearest-pixel .15m consistency diagnostic, not a source gate or proof across sequence; native grayscale window Hamming permutation-equivalent; no parameter/model change; no GT online'))
print(summary)
