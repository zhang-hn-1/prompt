from pathlib import Path
import json,copy,numpy as np
from exposure_contract import snapshot,paired_snapshot,local_transform
D=Path(__file__).resolve().parent
class Fake:
 def __init__(self,a):self.a=a
 def get_current_frame(self):return self.a
T=local_transform([.45,0,.05],0);f=584.8888888888889/3
base=dict(rendering_frame=2,rendering_time=.5,rgba=np.zeros((480,640,4),np.uint8),distance_to_image_plane=np.full((480,640),np.inf,np.float32),research_camera_params=dict(cameraViewTransform=np.linalg.inv(T.T).flatten(),renderProductResolution=np.array([640,480]),cameraFocalLength=16.45/3,cameraAperture=np.array([18.,13.5]),metersPerSceneUnit=1.))
r=snapshot(Fake(base),1,.5,f);assert np.allclose(r['camera_usd_world_label'],T,atol=1e-12);base['rgba'][0,0,0]=1;assert r['rgb'][0,0,0]==0;assert np.isinf(r['depth_label']).all();checks=['known_renderer_view_inverse','copied_RGB','inf_stays_unknown','paired_current_frames'];paired_snapshot([Fake(base),Fake(base)],[1,1],.5,f)
for name,mut in [('stale',lambda a:a.update(rendering_frame=1)),('time',lambda a:a.update(rendering_time=.48)),('missing',lambda a:a.pop('distance_to_image_plane')),('focal',lambda a:a['research_camera_params'].update(cameraFocalLength=16.45)),('units',lambda a:a['research_camera_params'].update(metersPerSceneUnit=.01)),('shape',lambda a:a.update(rgba=np.zeros((640,480,4),np.uint8))),('nan',lambda a:a['distance_to_image_plane'].__setitem__((0,0),np.nan))]:
 a=copy.deepcopy(base);mut(a)
 try:snapshot(Fake(a),1,.5,f)
 except ValueError:checks.append('rejected_'+name)
 else:raise AssertionError(name)
(D/'contract_qualification.json').write_text(json.dumps(dict(pass_gate=True,checks=checks,scope='Pure deterministic SDK contract fixtures, no real exposure/runtime admission. Numeric/model/old budgets unchanged.'),indent=2)+'\n');print(checks)
