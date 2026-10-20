import json
import numpy as np
import pytest
import learned_memory_risk_dataset_v1 as data
from learned_memory_risk_model_v1 import predict_features
from learned_memory_risk_features_v1 import FEATURE_NAMES

def test_feature_only_reader_never_opens_truth_and_uses_native_disparity(tmp_path,monkeypatch):
 for folder in ['partial_observations','source_inputs','source_predictions','frames']:(tmp_path/folder).mkdir()
 p=tmp_path/'partial_observations/frame_005.npz'
 np.savez(p,frame=5,acquisition_time_s=.5,world=[[0.,0.,.1]],query_pixel_id=np.array([640]),partner_pixel_id=np.array([641]),sigma_m=[.02])
 np.savez(tmp_path/'source_inputs'/p.name,time_s=.5,left_rgb=np.zeros((480,640,3),np.uint8),right_rgb=np.ones((480,640,3),np.uint8))
 np.savez(tmp_path/'source_predictions'/p.name,time_s=.5,frame=5,depth_m=np.ones((480,640)),disparity_px=np.full((640,480),2.))
 monkeypatch.setattr(data,'verify_sequence',lambda D:35.)
 original=np.load
 def protected(path,*args,**kwargs):
  assert '/frames/' not in str(path),'teacher opened by feature pipeline'
  return original(path,*args,**kwargs)
 monkeypatch.setattr(data.np,'load',protected)
 X,index,inputs=data.feature_only_sequence(tmp_path)
 assert X.shape==(1,23) and X[0,14]==1 and X[0,15]==2
 assert index==[dict(frame=5,time_s=.5,query_pixel_id=640)]
 assert len(inputs)==3 and all('/frames/' not in p for p in inputs)

def test_teacher_camera_frame_ground_height_and_invalid_exclusion(tmp_path):
 (tmp_path/'frames').mkdir();T=np.eye(4);T[2,3]=1
 depth=np.full((480,640),np.nan);depth[239,319]=1.;depth[240,319]=.8
 np.savez(tmp_path/'frames/frame_005.npz',time_s=.5,left_depth_m=depth,left_camera_usd_world=T)
 index=[dict(frame=5,time_s=.5,query_pixel_id=y*640+319) for y in [239,240,241]]
 y,h,inputs=data.offline_teacher_labels(tmp_path,index)
 assert y.tolist()==[0,1,-1] and abs(h[0])<1e-12 and h[1]==pytest.approx(.2)
 assert len(inputs)==1
 with pytest.raises(AssertionError):data.offline_teacher_labels(tmp_path,[dict(frame=5,time_s=.6,query_pixel_id=239*640+319)])

def test_predictor_uses_frozen_normalizer_and_rejects_schema_or_nan():
 model=dict(feature_names=FEATURE_NAMES,mean=[1.]*23,scale=[2.]*23,coef=[0.]*22+[2.],intercept=0.,threshold=.5)
 X=np.ones((2,23));X[1,-1]=3
 p,keep=predict_features(model,X)
 assert p[0]==.5 and p[1]==pytest.approx(1/(1+np.exp(-2))) and keep.tolist()==[True,True]
 with pytest.raises((AssertionError,ValueError)):predict_features(model,np.full((1,23),np.nan))
 with pytest.raises((AssertionError,ValueError)):predict_features(dict(model,feature_names=['teacher']),X)

def test_prefit_identity_detects_same_names_changed_semantics_before_fit(tmp_path):
 from learned_memory_risk_identity_v1 import verify_prefit_identity,sha
 source=tmp_path/'features.py';source.write_text('FEATURE_NAMES=["observed_depth_m"]; value=depth')
 identity=tmp_path/'learned_memory_risk_prefit_identity_v2.json';identity.write_text(json.dumps(dict(fit_not_executed_at_freeze=True,fresh_teacher_not_inspected_at_freeze=True,input_sha256={str(source):sha(source)})))
 assert verify_prefit_identity(tmp_path)==sha(identity)
 source.write_text('FEATURE_NAMES=["observed_depth_m"]; value=future_depth')
 with pytest.raises(AssertionError):verify_prefit_identity(tmp_path)

def test_two_frozen_historical_seals_require_explicit_external_normal_close(tmp_path):
 D=tmp_path/'sgbm_baseline35_e2_run_v1';D.mkdir()
 (D/'artifact_manifest_post_close.json').write_text(json.dumps(dict(artifact_sha256={})))
 (D/'identity.json').write_text(json.dumps(dict(complete=True,reset_count=1,internal_reset=False)))
 (D/'physics_contacts.json').write_text(json.dumps([dict(available=True,contact=False,simulation_time_s=35.)]))
 p=tmp_path/'sgbm_baseline35_e2_status_v1.json';base=dict(status='finished',exit_code=0,normal_close_confirmed=True,child_execution=dict(returncode=0,timeout=False,forced_kill=False));p.write_text(json.dumps(base))
 assert data.verify_sequence(D)==35.
 base['child_execution']['timeout']=True;p.write_text(json.dumps(base))
 with pytest.raises(AssertionError):data.verify_sequence(D)
