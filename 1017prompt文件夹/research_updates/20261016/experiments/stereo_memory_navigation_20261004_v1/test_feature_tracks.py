"""Catch wrong stereo signs, weak baseline, fabricated future input and stale/free memory."""
import numpy as np
import feature_tracks as f
K=np.array([[100.,0,64],[0,100.,64],[0,0,1.]])
A=np.diag([1.,-1.,-1.,1.]);B=A.copy();B[0,3]=.2

def test_triangulation_hand_two_metre_point_and_uncertainty():
 r=f.triangulate([64,64],[54,64],A,B,K)
 assert r is not None
 np.testing.assert_allclose(r['world'],[0,0,2],atol=1e-12)
 assert 5<r['parallax_deg']<6 and 0<r['sigma_m']<.5

def test_same_camera_and_negative_depth_are_not_valid_points():
 assert f.triangulate([64,64],[64,64],A,A,K) is None
 assert f.triangulate([64,64],[74,64],A,B,K) is None

def test_epipolar_distance_hand_horizontal_baseline():
 d=f.epipolar_distances(np.array([[64,64]]),np.array([[54,64],[54,70]]),A,B,K)
 np.testing.assert_allclose(d,[[0,6]],atol=1e-12)

def test_future_payload_never_read():
 class Future(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future payload read')
   return 4.
 ref=dict(time_s=3.,frame=30,eye='left');current=dict(time_s=3.,frame=30,eye='right');past=dict(time_s=2.,frame=20,eye='left')
 assert f.causal_sources(ref,[current,past,Future()])==[current,past]

def test_entire_rotated_descriptor_support_rejects_mask_hole():
 mask=np.zeros((100,100),bool);mask[49,60]=True
 assert not f.full_support([50,50],31,mask)
 assert f.full_support([75,75],10,mask)
 assert not f.full_support([2,2],31,np.zeros((100,100),bool))

def test_unambiguous_mutual_descriptor_matches_and_equal_rivals_reject():
 q=np.array([[64,64],[64,80]],float);s=np.array([[54,64],[44,64],[54,80],[44,80]],float)
 qd=np.array([[0]*32,[255]*32],np.uint8);sd=np.array([[0]*32,[127]*32,[255]*32,[128]*32],np.uint8)
 assert f.match_descriptors(q,qd,s,sd,A,B,K)==[(0,0),(1,2)]
 sd[1]=sd[0];assert (0,0) not in f.match_descriptors(q,qd,s,sd,A,B,K)

def test_memory_keeps_provenance_expires_and_never_certifies_free():
 m=f.FeatureMemory(10)
 point=dict(world=[0,0,.5],sigma_m=.01,query=[0,'left',1,0.],source=[0,'right',2,0.])
 m.update([point],0,0);assert len(m.snapshot(1))==1 and m.snapshot(11)==[]
 assert m.snapshot(1)[0]['source']==[0,'right',2,0.]
 import pytest
 with pytest.raises(ValueError):m.update([point],-1,0)
 m.update([point],2,1);assert m.snapshot(2)==[]
 assert m.certified_free([0,0],2) is False

def test_memory_snapshot_cannot_modify_retained_source_evidence():
 m=f.FeatureMemory();p=dict(world=[0,0,.5],sigma_m=.01,query=[0,'left',1,0.],source=[0,'right',2,0.]);m.update([p],0,0)
 view=m.snapshot(1);view[0]['world'][2]=99;view[0]['source'][0]=99
 assert m.snapshot(1)[0]['world']==[0,0,.5] and m.snapshot(1)[0]['source'][0]==0

def test_prediction_source_selection_is_causal_and_world_points_hand_derived():
 k=np.array([[1000.,0,64],[0,1000.,64],[0,0,1.]])
 ref=dict(frame=30,time_s=3.,eye='left',pose=A,xy=np.array([[64,64],[64,80]],float),desc=np.array([[0]*32,[255]*32],np.uint8))
 src=dict(frame=30,time_s=3.,eye='right',pose=B,xy=np.array([[-36,64],[-136,64],[-36,80],[-136,80]],float),desc=np.array([[0]*32,[127]*32,[255]*32,[128]*32],np.uint8))
 class Future(dict):
  def __getitem__(self,key):
   if key!='time_s':raise AssertionError('future data leaked')
   return 4.
 r=f.predict_features(ref,[src,Future()],k);assert len(r['points'])==2
 np.testing.assert_allclose([p['world'] for p in r['points']],[[0,0,2],[0,.032,2]],atol=1e-10)
 assert r['source_views']==[[30,'right',3.]]
 import pytest
 m=f.FeatureMemory()
 with pytest.raises(ValueError):m.update([dict(r['points'][0],source=[40,'right',2,4.])],3.,0)
