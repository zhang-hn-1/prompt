import unittest,numpy as np,json,tempfile
from pathlib import Path
from four_camera import NAMES,nominal_transforms,validate_packets,validate_complete
class Contract(unittest.TestCase):
 def packets(self):
  ts=nominal_transforms(.2,.3,.05);B=np.eye(4);B[:3,3]=[2,3,.4]
  return [dict(render_frame=10,render_time=.5,camera_usd_world_label=B@t,rgb=np.zeros((480,640,3),np.uint8),depth_label=np.ones((480,640))) for t in ts]
 def test_original_front(self):
  t=nominal_transforms(.2,.3,.05);np.testing.assert_allclose([a[:3,3] for a in t],[[.2,.3,.05],[.2,-.3,.05],[-.2,.3,.05],[-.2,-.3,.05]])
 def test_axes(self):
  ts=nominal_transforms(.2,.3,.05)
  for i,t in enumerate(ts):np.testing.assert_allclose(t[:3,:3]@[0,0,-1],[1 if i<2 else -1,0,0],atol=1e-12);self.assertAlmostEqual(np.linalg.det(t[:3,:3]),1)
 def test_four(self):self.assertLess(validate_packets(self.packets(),[-1]*4,.5,nominal_transforms(.2,.3,.05)),1e-12)
 def test_missing(self):
  with self.assertRaises(ValueError):validate_packets(self.packets()[:2],[-1]*4,.5,nominal_transforms(.2,.3,.05))
 def test_stale(self):
  with self.assertRaises(ValueError):validate_packets(self.packets(),[10]*4,.5,nominal_transforms(.2,.3,.05))
 def test_skew(self):
  p=self.packets();p[3]['render_time']=.52
  with self.assertRaises(ValueError):validate_packets(p,[-1]*4,.5,nominal_transforms(.2,.3,.05))
 def test_wrong_rear(self):
  p=self.packets();p[3]['camera_usd_world_label'][0,3]+=.1
  with self.assertRaises(ValueError):validate_packets(p,[-1]*4,.5,nominal_transforms(.2,.3,.05))
 def test_zero_without_seal(self):
  with tempfile.TemporaryDirectory() as d:
   with self.assertRaises(ValueError):validate_complete(dict(status='finished',child_returncode=0),Path(d))
 def test_missing_terminal_rejected(self):
  import hashlib
  with tempfile.TemporaryDirectory() as dd:
   d=Path(dd);(d/'frames').mkdir();rows=[]
   for i in range(8):
    f=d/'frames'/f'{i}.npz';np.savez(f,**{n+'_rgb':np.zeros((480,640,3),np.uint8) for n in NAMES})
    rows.append(dict(frame=f.name,sha256=hashlib.sha256(f.read_bytes()).hexdigest(),imu={'qualified':True},pair_error=0,render_frames=[i]*4))
   (d/'capture_seal.json').write_text(json.dumps(dict(complete=True,camera_names=list(NAMES),rows=rows)))
   with self.assertRaises(ValueError):validate_complete(dict(status='finished',child_returncode=0),d)
unittest.main()
