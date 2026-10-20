"""Literal consumer fixtures: missing support, time provenance, no floor invention."""
import numpy as np
import pytest
import point_support_shadow as module

def point(xyz=(0.,0.,0.),t=0.,index=0,sigma=.01):
 return dict(world=list(xyz),sigma_m=sigma,query=[int(t*10),'left',index,t],source=[int(t*10),'right',index,t])

def test_missing_consumer_cannot_invent_floor_between_sparse_points():
 # Break caught: filling a bounding region/interpolating unseen cells.
 m=module.PointSupportMap([0.,0.],(5,5),1.)
 m.update([point(),point((0.,4.,0.),index=1)],0.,0.)
 s=m.snapshot(0.)
 assert s['labels'][0].tolist()==[0,-1,-1,-1,0]
 assert (s['labels']==0).sum()==2
 assert (s['labels']==-1).sum()==23
 assert not s['certified_free']

def test_uncertainty_cannot_create_nominal_floor_support():
 # Break caught: ignoring 3D sensitivity or estimated-pose uncertainty.
 m=module.PointSupportMap([0.,0.],(5,5),.1)
 m.update([point((.2,.2,.045),sigma=.01),point((.3,.3,0.),index=1,sigma=.06)],0.,0.)
 assert (m.snapshot(0.)['labels']==0).sum()==0
 n=module.PointSupportMap([0.,0.],(5,5),.1)
 n.update([point()],0.,.06)
 assert n.snapshot(0.)['labels'][0,0]==-1

def test_past_hazard_dominates_floor_without_refreshing_its_time():
 # Break caught: forgetting an occluded hazard, floor overwriting, or replay refreshing age.
 m=module.PointSupportMap([0.,0.],(3,3),1.)
 hazard=point((1.,1.,1.),t=0.)
 m.update([hazard],0.,0.)
 m.update([point((1.,1.,0.),t=1.,index=1)],1.,0.)
 assert m.snapshot(1.)['provenance'][1,1]==4
 assert m.snapshot(10.)['labels'][1,1]==1
 m.update([hazard],10.1,0.)
 assert m.snapshot(10.1)['labels'][1,1]==0
 assert m.snapshot(11.1)['labels'][1,1]==-1

def test_invalid_or_future_provenance_is_rejected_before_payload_or_mutation():
 # Break caught: NaN bypass, reading future payload, or corrupting state on invalid update.
 class Trap(dict):
  def __getitem__(self,key):
   if key not in ['query','source']:raise AssertionError('future payload accessed')
   return super().__getitem__(key)
 m=module.PointSupportMap([0.,0.],(3,3),1.)
 m.update([point()],0.,0.)
 m.update([Trap(query=[10,'left',0,1.],source=[10,'right',0,1.])],0.,0.)
 assert m.snapshot(0.)['labels'][0,0]==0
 bad=point((1.,1.,0.),index=2);bad['source'][3]=np.nan
 with pytest.raises(ValueError):m.update([point((2.,2.,0.),index=3),bad],0.,0.)
 assert (m.snapshot(0.)['labels']==0).sum()==1
 with pytest.raises(ValueError):m.snapshot(np.nan)
 with pytest.raises(ValueError):m.snapshot(-1.)

def test_whole_sweep_checks_midsegment_and_matching_stop_region():
 # Break caught: endpoint-only check or omitting stopping tail.
 cells=module.swept_indices([[0.,0.],[0.,4.],[4.,4.]],[0.,0.],1.,0.)
 assert set(map(tuple,cells))=={(0,0),(0,1),(0,2),(0,3),(0,4),(1,4),(2,4),(3,4),(4,4)}
 labels=np.zeros((5,5),np.int8);labels[2,4]=1
 r=module.support_result(labels,cells)
 assert r['checked_cells']==9 and r['occupied_cells']==1 and not r['support_passed']
 labels[2,4]=-1;r=module.support_result(labels,cells)
 assert r['unknown_cells']==1 and not r['support_passed']
 outside=module.swept_indices([[-1.,0.],[0.,0.]],[0.,0.],1.,0.)
 assert module.support_result(labels,outside)['unknown_cells']==1

def test_prefix_replay_does_not_read_future_packet_and_never_certifies_actions():
 # Break caught: using full saved sequence before current time or admitting rejected points.
 class Trap(dict):
  def __getitem__(self,key):
   if key!='time_s':raise AssertionError('future packet accessed')
   return super().__getitem__(key)
 packets=[dict(time_s=0.,points=[point()],pose_sigma_m=0.),Trap(time_s=1.)]
 m=module.replay_prefix(packets,0.,[0.,0.],(3,3),1.)
 assert m.snapshot(0.)['labels'][0,0]==0
 assert (m.snapshot(0.)['labels']==-1).sum()==8

def test_archived_plan_identity_accepts_only_exact_original_and_recorded_closure():
 # Break caught: blindly skipping a changed plan or accepting an unrecorded suffix.
 import hashlib
 import verify_point_support_shadow as verifier
 original=b'plan v1\n';closed=original+b'\n\n## 00:36UTC original-scene active observation\nclosure\n'
 expected=hashlib.sha256(original).hexdigest();final=hashlib.sha256(closed).hexdigest()
 assert verifier.archived_plan_identity(closed,expected,final)==original
 with pytest.raises(ValueError):verifier.archived_plan_identity(closed+b'changed',expected,final)
 with pytest.raises(ValueError):verifier.archived_plan_identity(closed.replace(b'v1',b'v2'),expected,hashlib.sha256(closed.replace(b'v1',b'v2')).hexdigest())
