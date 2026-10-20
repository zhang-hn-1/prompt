from pathlib import Path
import sys,numpy as np
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D.parent/'stereo_memory_navigation_20261004_v1'))
from dynamic_self_mask import RobotSelfMask
from self_mask import mask_triangles
from four_camera import nominal_transforms,NAMES
class FourCAD:
 def __init__(self):self.cad=RobotSelfMask();self.transforms=nominal_transforms(.2,.3,.05)
 def masks(self,joints):
  body=self.cad.body_transforms(joints);v=np.concatenate([t@body[k][:3,:3].T+body[k][:3,3] for k,t in self.cad.triangles.items()]);out={}
  for name,T in zip(NAMES,self.transforms):
   optical=(v-T[:3,3])@(T[:3,:3]@np.diag([1,-1,-1]));out[name]=mask_triangles(optical,640,480,584.8888888888889,319.5,239.5)
  return out
if __name__=='__main__':
 import json,time
 t=time.monotonic();m=FourCAD();masks=m.masks(np.zeros(3));np.savez_compressed(D/'initial_CAD_masks.npz',**masks);result=dict(counts={k:int(v.sum()) for k,v in masks.items()},pixels_per_camera=307200,scope='Nominal q=0 CAD silhouette, no scene/ground truth or calibration guarantee',identity=m.cad.identity,wall_s=time.monotonic()-t);(D/'initial_CAD_check.json').write_text(json.dumps(result,indent=2)+'\n');print(result['counts'],flush=True)
