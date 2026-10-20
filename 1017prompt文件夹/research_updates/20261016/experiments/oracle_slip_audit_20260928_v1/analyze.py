from pathlib import Path
import json,numpy as np
from scipy.spatial.transform import Rotation
h=Path(__file__).resolve().parent;p=h/'LR_candidate';z=np.load(p/'links.npz');q=z['poses'][:,0,1];v=z['velocities'][:,0,1];rot=Rotation.from_quat(q[:,3:7]);offset=rot.apply(np.tile([.010809,.020684,.012501],(len(v),1)));vc=v[:,:3]-np.cross(v[:,3:],offset);slip=vc+np.cross(v[:,3:],np.tile([0.,0.,-.4000000059604645],(len(v),1)));s=np.linalg.norm(slip[:,:2],axis=1);speed=np.linalg.norm(vc[:,:2],axis=1);t=np.arange(len(v))*.02;mask=(t>=1)&(q[:,2]<.401);moving=mask&(speed>.05)
def stats(x):return {'n':len(x),'median':float(np.median(x)),'p95':float(np.quantile(x,.95)),'max':float(x.max())}
# Differentiating poses is only an independent velocity-frame consistency check.
fd=np.gradient(q[:,:3],.02,axis=0);res=fd-vc
out={'body':'link1','radius_m':.4000000059604645,'com_local_m':[.010809,.020684,.012501],'settled_geometry_filter':'t>=1s and sphere center z<.401m; not a ground force measurement','contact_tangent_residual_m_s':stats(s[mask]),'moving_residual_m_s':stats(s[moving]),'moving_residual_over_center_speed':stats(s[moving]/speed[moving]),'samples_residual_over_0_01_m_s':int((s[mask]>.01).sum()),'center_velocity_finite_difference_error_m_s':stats(np.linalg.norm(res[mask],axis=1)),'phases':{}}
for a,b in [(2,8),(8,12),(21.5,29),(30,35)]:out['phases'][f'{a}-{b}s']=stats(s[(t>=a)&(t<b)])
(h/'assessment.json').write_text(json.dumps(out,indent=2));np.savez(h/'rolling_residual.npz',time=t,center_velocity=vc,contact_tangent_velocity=slip,settled_mask=mask);print(json.dumps(out,indent=2))
