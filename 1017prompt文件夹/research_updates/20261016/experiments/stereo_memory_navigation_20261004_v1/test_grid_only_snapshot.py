import numpy as np,pytest
from point_support_shadow import PointSupportMap
from grid_only_snapshot import grid_arrays_snapshot
def point(x,z,t=0,sigma=0):return dict(world=[x,0,z],sigma_m=sigma,query=[int(t*10),'left',int(x*10+20+z*100),t],source=[int(t*10),'right',1,t])
def test_literal_floor_hazard_conflict_times_and_no_rawrecords_copy():
 m=PointSupportMap([0,0],(4,3),.1);m.update([point(.1,0),point(.2,.3),point(.1,.2)],0,0);r=grid_arrays_snapshot(m,0);assert 'records' not in r;assert r['labels'][1,0]==1 and r['labels'][2,0]==1 and r['conflict_cells']==1;assert r['provenance'][1,0]==3 and r['hazard_time'][1,0]==0 and r['floor_time'][1,0]==0
def test_history_exact_ttl_rounding_outbounds_uncertainty_and_ignored():
 m=PointSupportMap([0,0],(4,3),.1);m.update([point(.1,0),point(4,.2),point(.2,-.2)],0,0);r=grid_arrays_snapshot(m,10);assert r['labels'][1,0]==0 and r['provenance'][1,0]==2 and r['ignored_records']==2;r=grid_arrays_snapshot(m,10.000001);assert (r['labels']==-1).all() and r['ignored_records']==0
def test_full_original_snapshot_fields_equal_and_no_state_mutation():
 import copy
 m=PointSupportMap([0,0],(4,3),.1);m.update([point(.1,0),point(.2,.3),point(.3,.04,sigma=.02)],0,0);before=copy.deepcopy(m.records);a=m.snapshot(.1);b=grid_arrays_snapshot(m,.1);assert before==m.records
 for k,v in b.items():np.testing.assert_equal(a[k],v)
def test_invalid_query_rejected_before_reading_records():
 class Bomb:
  time=2
  @property
  def records(self):raise AssertionError('payload')
 for t in (1,float('nan')):
  with pytest.raises(ValueError):grid_arrays_snapshot(Bomb(),t)
