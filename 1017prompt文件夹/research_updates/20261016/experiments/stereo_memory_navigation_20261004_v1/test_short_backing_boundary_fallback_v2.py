from dataclasses import dataclass
from types import SimpleNamespace as S
import numpy as np
import pytest

@dataclass
class Waypoint:
 drive_direction:str
@dataclass
class Plan:
 status:str
 waypoints:tuple

@pytest.mark.parametrize('alternative_kind',['missing','same_backing','longer_backing'])
def test_real_composed_boundary_falls_back_without_forced_state(monkeypatch,alternative_kind):
 import minimum_prefix_transition_consumer_v1 as first
 import continuous_cell_tube_consumer_v1 as tube
 from short_initial_backing_v1 import with_short_initial_backing
 calls=[];installed=[]
 original=Plan('success',tuple(Waypoint(x) for x in ['reverse']*5+['forward']*3))
 alternative=None if alternative_kind=='missing' else Plan('success',tuple(Waypoint(x) for x in ['reverse']*(5 if alternative_kind=='same_backing' else 6)+['forward','forward']))
 def callback(grid,xy,yaw,goal,drive,**kw):
  calls.append((grid,kw.copy()))
  p=original if len(calls)==1 else alternative
  return p,dict(status='success',expanded_nodes=3)
 monkeypatch.setattr(first,'local_curve_first_direction',callback)
 monkeypatch.setattr(tube,'reference_geometry_status',lambda *a:dict(retention_necessary_screen_pass=False))
 monkeypatch.setattr(tube,'install_local_reference',lambda tracker,p:installed.append(p))
 class Empty:pass
 cls=with_short_initial_backing(first.with_first_primitive_direction(tube.with_continuous_cell_tube_after_measured_reversal(Empty)))
 obj=object.__new__(cls)
 tracker=S(_path_index=7,_active_drive_direction='forward',_pending_drive_direction=None,_stopped_cycles=2,_off_reference_cycles=0,_failure_subreason=None,_braking_state='tracking',mapping_records=[],_reference_xy=np.array([[0.,0.],[1.,0.]]),_reference=(S(drive_direction='forward'),)*2,_context=object(),drive_model=object())
 obj.tracker=tracker;obj.plan=original;obj.local_reference_version=1;obj.tube_viability_armed=False;obj.current_first_direction=None;obj.first_direction_query=None;obj.short_backing_alternative_queries=0
 grid=S(labels=np.full((40,40),-1,dtype=np.int8),origin=np.array([-2.,-2.]),resolution=.1,time=15.5,hazard_acquisition_time=np.zeros((40,40)),commitment=dict(decision_time_s=6.,axis_start=[0.,0.],axis=[1.,0.],side=-1,longitudinal_span_m=[1.,2.]))
 obj.grid=grid;before=grid.labels.copy();hint=grid.commitment.copy()
 created,d=obj.plan_boundary(np.array([.5,0.]),0.,np.array([4.,0.]),15.5)
 assert created and d['decision']=='rebuild' and obj.plan is original and installed==[original]
 assert len(calls)==2 and calls[0][0] is grid and calls[1][0] is not grid
 assert calls[0][1]==calls[1][1] and calls[0][1]['first_primitive_direction'] is None
 assert tracker._path_index==7 and tracker._active_drive_direction=='forward' and tracker._pending_drive_direction is None and tracker._stopped_cycles==2
 assert np.array_equal(grid.labels,before) and grid.commitment==hint
 assert first.local_curve_first_direction is callback
 assert not obj.short_backing_this_call['chosen_optional'] and obj.short_backing_alternative_queries==1
 obj.plan_boundary(np.array([.5,0.]),0.,np.array([4.,0.]),16.)
 assert len(calls)==3 and obj.short_backing_alternative_queries==1
