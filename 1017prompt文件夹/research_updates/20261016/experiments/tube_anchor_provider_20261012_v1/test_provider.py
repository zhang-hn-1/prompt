import numpy as np,pytest
from provider import provide,FlatGroundPoseBridge
class Receiver:
 def anchor(self,*args,**kw):self.args=args;self.kw=kw

def pose():
 p=FlatGroundPoseBridge([0,0,.4],np.eye(3));p.consume(dict(time_s=1.,position=[0,0,0],rotation=np.eye(3),velocity=[.1,0,0],shell_omega_gauge=[0,0,0]));return p

def inputs():return dict(time_s=1.,raw_time_s=11.,left_render_time_s=11.,right_render_time_s=11.,left_rgb=np.zeros((480,640,3),np.uint8),right_rgb=np.ones((480,640,3),np.uint8))
def test_typed_source():
 r=Receiver();receipt=provide(r,pose(),**inputs());assert r.kw['source']=='native_imu_rolling' and r.kw['estimated_velocity']==(.1,0.);assert receipt['raw_time_s']==11.
def test_wrong_pose_type():
 with pytest.raises(ValueError):provide(Receiver(),object(),**inputs())
def test_stale_camera():
 d=inputs();d['right_render_time_s']=10.9
 with pytest.raises(ValueError):provide(Receiver(),pose(),**d)
def test_bad_rgb():
 d=inputs();d['left_rgb']=np.zeros((480,640,3),float)
 with pytest.raises(ValueError):provide(Receiver(),pose(),**d)
def test_future_pose_unavailable():
 d=inputs();d['time_s']=1.02
 with pytest.raises(ValueError):provide(Receiver(),pose(),**d)
def test_copied_rgb():
 r=Receiver();d=inputs();provide(r,pose(),**d);d['left_rgb'][:]=255;assert r.kw['left_rgb'][0]==0

def test_runtime_epoch_binding():
 from provider import provide_clocked
 class Camera:
  def get_current_frame(self):return {'rendering_time':11.}
 class Runtime:
  _left_camera=Camera();_right_camera=Camera()
  def _clock_metadata(self):return (1.,50,11.,550)
 r=Receiver();receipt=provide_clocked(r,pose(),Runtime(),lambda c:np.zeros((480,640,3),np.uint8));assert receipt['clock_metadata']==[1.,50,11.,550]
def test_runtime_advanced_during_read():
 from provider import provide_clocked
 class Camera:
  def get_current_frame(self):return {'rendering_time':11.}
 class Runtime:
  _left_camera=Camera();_right_camera=Camera();calls=0
  def _clock_metadata(self):self.calls+=1;return (1.,50,11.,550) if self.calls==1 else (1.02,51,11.02,551)
 with pytest.raises(ValueError):provide_clocked(Receiver(),pose(),Runtime(),lambda c:np.zeros((480,640,3),np.uint8))
