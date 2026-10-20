"""Catch masked descriptors, future reads and geometrically inconsistent learned pairs."""
import numpy as np
import pytest
from learned_correspondence import supported_features,predict_correspondences
A=np.diag([1.,-1.,-1.,1.]);B=A.copy();B[0,3]=.2
K=np.array([[1000.,0,200],[0,1000.,200],[0,0,1.]])
def views():
 return dict(frame=30,time_s=3.,eye='left',pose=A,xy=np.array([[200.,200.]])),dict(frame=30,time_s=3.,eye='right',pose=B,xy=np.array([[100.,200.]]))
def test_entire_learned_support_not_just_centre_is_masked():
 mask=np.zeros((400,400),bool);mask[200,135]=True
 f={'xy':np.array([[200.,200.],[300.,300.],[5.,5.]]),'desc':np.ones((3,256)),'scores':np.array([.9,.8,.7])}
 r=supported_features(f,mask)
 assert r['xy'].tolist()==[[300.,300.]] and r['original_indices'].tolist()==[1]
 r['desc'][0,0]=10;assert f['desc'][1,0]==1
def test_geometric_point_from_actual_pairs_and_causal_future_trap():
 q,s=views()
 class Future(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('futurepayload')
   return 4.
 r=predict_correspondences(q,[s,Future()],K,lambda a,b:np.array([[0,0]]))
 assert r['query_count']==1 and len(r['points'])==1
 np.testing.assert_allclose(r['points'][0]['world'],[0,0,2],atol=1e-10)
 assert r['source_views']==[[30,'right',3.]]
def test_bad_epipolar_and_negative_depth_reject():
 q,s=views();s['xy'][0,1]=210
 assert predict_correspondences(q,[s],K,lambda a,b:np.array([[0,0]]))['points']==[]
 s['xy'][0]=[300,200]
 assert predict_correspondences(q,[s],K,lambda a,b:np.array([[0,0]]))['points']==[]
def test_conflicting_causal_sources_preserve_unknown():
 q,s=views();p=dict(s,frame=20,time_s=2.,xy=np.array([[150.,200.]]))
 r=predict_correspondences(q,[s,p],K,lambda a,b:np.array([[0,0]]))
 assert r['points']==[] and r['conflicting_queries']==1
@pytest.mark.parametrize('bad',[np.array([[1,0]]),np.array([[0,0],[0,0]]),np.array([[0.1,0.]])])
def test_invalid_pair_indices_are_rejected(bad):
 q,s=views()
 with pytest.raises(ValueError):predict_correspondences(q,[s],K,lambda a,b:bad)
def test_nonfinite_reference_time_rejected_before_matching():
 q,s=views();q['time_s']=float('nan')
 with pytest.raises(ValueError):predict_correspondences(q,[s],K,lambda a,b:np.array([[0,0]]))
