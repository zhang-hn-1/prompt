from pathlib import Path
import json,tempfile,hashlib
import numpy as np
from confidence import SourceConfidence
D=Path(__file__).resolve().parent;rgb=dict(left_rgb=np.zeros((480,640,3),np.uint8),right_rgb=np.zeros((480,640,3),np.uint8),time_s=np.array(1.));pred=dict(depth_m=np.ones((480,640),np.float32),disparity_px=np.ones((640,480),np.float32)*350,frame=np.array(10),time_s=np.array(1.));packet=dict(world=np.array([[0.,0.,.2],[1.,0.,.3]]),sigma_m=np.ones(2)*.01,query_pixel_id=np.array([640*100+100,640*101+100]),partner_pixel_id=np.array([640*101+100,640*102+100]),frame=np.array(10),time_s=np.array(1.),certified_free=np.array(False));before={k:v.copy() for k,v in packet.items()};passed=[]
with tempfile.TemporaryDirectory(dir='/tmp') as tmp:
 p=Path(tmp)/'synthetic_fixture.npz';w=np.zeros(13);w[0]=-1;np.savez(p,mean=np.zeros(12),std=np.ones(12),weights=w);m=SourceConfidence(p);r=m.qualify_packet(rgb,pred,packet,1.);assert len(r['world'])==2;passed.append('aligned_keep')
 w[0]=1;np.savez(p,mean=np.zeros(12),std=np.ones(12),weights=w);m=SourceConfidence(p);r=m.qualify_packet(rgb,pred,packet,1.);assert len(r['world'])==0 and not bool(r['certified_free']) and bool(r['excluded_is_unknown']);passed.append('reject_is_unknown')
 for args in [(rgb,pred,{**packet,'time_s':np.array(2.)},1.),(rgb,{**pred,'frame':np.array(11)},packet,1.),(rgb,pred,{**packet,'partner_pixel_id':np.array([-1,20])},1.)]:
  try:m.qualify_packet(*args)
  except ValueError:passed.append('invalid_input_rejected')
  else:raise AssertionError('poison accepted')
 for k,v in before.items():np.testing.assert_array_equal(v,packet[k])
 passed.append('source_immutable')
(D/'interface_checks.json').write_text(json.dumps(dict(passed=passed,count=len(passed),model='synthetic intercept-only fixture; not a training fit',fit=0,files_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'confidence.py',D/'features.py',D/'check_interface.py']}),indent=2)+'\n');print(len(passed),'checks')
