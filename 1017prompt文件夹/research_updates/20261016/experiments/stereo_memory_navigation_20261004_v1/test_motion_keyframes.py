"""Catch selecting future images, tiny baseline, stale/rotated views or duplicate baselines."""
import numpy as np
import pytest
from motion_keyframes import select_keyframes
def pose(x=0):
 p=np.eye(4);p[0,3]=x;return p
def row(frame,t,x):return dict(frame=frame,time_s=t,pose=pose(x))
def test_newest_diverse_baselines_and_no_short_fallback():
 old=[row(1,1,.55),row(2,2,.4),row(3,3,.25),row(4,4,.24),row(5,5,.10)]
 assert [p['frame'] for p in select_keyframes(pose(),6,old)]==[4,2,1]
 assert select_keyframes(pose(),6,[row(5,5,.10)])==[]
def test_future_payload_is_never_read_and_stale_ignored():
 class Future(dict):
  def __getitem__(self,k):
   if k!='time_s':raise AssertionError('future read')
   return 21.
 assert select_keyframes(pose(),20,[Future(),row(1,9,.3)])==[]
def test_baseline_boundaries_rotation_and_deepcopy():
 near=row(1,1,.15);far=row(2,2,.6);outside=row(3,3,.601);rot=row(4,4,.3)
 a=np.radians(21);rot['pose'][:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]
 chosen=select_keyframes(pose(),5,[near,far,outside,rot]);assert [p['frame'] for p in chosen]==[2,1]
 chosen[0]['pose'][0,3]=10;assert far['pose'][0,3]==.6
@pytest.mark.parametrize('bad',[float('nan'),float('inf')])
def test_nonfinite_time_rejected(bad):
 with pytest.raises(ValueError):select_keyframes(pose(),bad,[])
