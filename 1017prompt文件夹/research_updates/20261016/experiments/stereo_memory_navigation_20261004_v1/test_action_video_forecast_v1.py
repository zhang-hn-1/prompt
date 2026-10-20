import numpy as np
import pytest
from action_video_forecast_v1 import causal_features, fit_once, predict

def test_current_time_and_prior_flow_clock_required():
 q=np.array([[320.,240.]])
 with pytest.raises(ValueError):causal_features(np.zeros(14),q,np.zeros((1,2)),[True],0,1.,1.1)
 with pytest.raises(ValueError):causal_features(np.zeros(14),q,np.zeros((1,2)),[True],0,float('nan'),float('nan'))
 x,b=causal_features(np.zeros(14),q,[[99,99]],[False],0,1.,1.)
 assert x.shape==(1,23);np.testing.assert_equal(b,0);np.testing.assert_equal(x[:,19:22],0)

def test_whole_train_scaling_and_residual_prediction_only():
 rng=np.random.default_rng(7);x=rng.normal(size=(100,23));base=np.ones((100,2));truth=base+np.column_stack([x[:,0]*.4,x[:,1]*.6]);m=fit_once(x,truth,base,np.ones(100))
 saved={k:v.copy() for k,v in m.items()};out=predict(m,x[:2],base[:2]);assert np.mean((out-truth[:2])**2)<.005
 predict(m,np.ones((2,23))*1e6,base[:2])
 for k in saved:np.testing.assert_equal(m[k],saved[k])
 assert np.all(m['scale']>=1)

def test_unavailable_prior_stays_zero_and_features_ignore_future():
 class Trap(dict):
  def __getitem__(self,k):
   if k=='future':raise AssertionError('future read')
   return super().__getitem__(k)
 a=Trap(sensor=np.zeros(14),future='poison')
 x,b=causal_features(a['sensor'],[[12,12]],[[np.nan,np.nan]],[False],1,.1,.1)
 assert np.isfinite(x).all();np.testing.assert_equal(b,0)

def test_future_image_replacement_and_log_truncation_leave_prefix_inputs_identical(tmp_path,monkeypatch):
 import json
 import run_action_video_forecast_fit_validation_v1 as stage
 class Capture:
  def __init__(self,images):self.images=images;self.i=0
  def read(self):
   if self.i==len(self.images):return False,None
   a=self.images[self.i];self.i+=1;return True,a.copy()
  def release(self):pass
 rng=np.random.default_rng(1);image=rng.integers(0,256,(480,1280,3),dtype=np.uint8);frames=[np.roll(image,i,axis=1) for i in range(6)]
 monkeypatch.setattr(stage,'H',tmp_path);out=tmp_path/'out';out.mkdir();monkeypatch.setattr(stage,'O',out);monkeypatch.setattr(stage,'MASK',{eye:np.zeros((480,640),bool) for eye in ('left','right')})
 def create(name,n,images,future_poison=False):
  d=tmp_path/name;d.mkdir();r=np.zeros((n,2));r[3:]=999 if future_poison else 0;np.savez(d/'requests.npz',requests=r);np.savez(d/'physics.npz',joint_position_rad=np.zeros((n*5+1,3)))
  states=[dict(time_s=i*.1) for i in range(n+1)];(d/'states.json').write_text(json.dumps(states));packets=[dict(time_s=i*.1,gyro_body_rad_s=[0,0,0],linear_acceleration_body_m_s2=[0,0,0],main_encoder_rad_s=0) for i in range(n+1)];(d/'native_imu_packets.json').write_text(json.dumps(packets));monkeypatch.setattr(stage.cv2,'VideoCapture',lambda path:Capture(images));return stage.build_inputs(name)[0]
 full=create('full',5,frames);poisoned=create('poisoned',5,frames[:3]+[np.zeros_like(image)]*3,True);cut=create('cut',3,frames[:3])
 for k in ('features','baseline','moving','indices','times_s'):
  np.testing.assert_equal(full[k][:2],poisoned[k][:2]);np.testing.assert_equal(full[k][:2],cut[k])
