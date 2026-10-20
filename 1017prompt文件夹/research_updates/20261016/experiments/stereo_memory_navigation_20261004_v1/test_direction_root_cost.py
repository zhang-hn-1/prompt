"""Literal public behavior fixtures; no archived-frame searches in tests."""
from pathlib import Path
import sys
from dataclasses import asdict,replace
import numpy as np
import pytest
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from benchmark import kinodynamic_planning as old
from benchmark.drive_model import DriveModel
from benchmark.policies import DEFAULT_DRIVE_PROFILE
import direction_root_planning as new
import direction_root_cost as m

def model():return DriveModel.from_json(DEFAULT_DRIVE_PROFILE)
def request(goal=(-.6,0.),switch=.5):
 return new.KinodynamicPlanRequest(np.zeros((61,61),np.uint8),.1,(-3.,-3.),(0.,0.),0.,goal,node_budget=3000,direction_change_cost_m=switch)
def toy_plan(directions):
 points=tuple(old.KinodynamicWaypoint(-i*.3,0.,0.,d,.5 if d=='forward' else -.5,0.) for i,d in enumerate(directions))
 points+=(old.KinodynamicWaypoint(-len(directions)*.3,0.,0.,directions[-1],0.,0.),)
 return old.KinodynamicPlanResult('success',points,1,len(directions)*.3,sum(a!=b for a,b in zip(directions,directions[1:])))

@pytest.mark.parametrize('directions,initial,total,initial_change,inside',[
 (['reverse']*3,'forward',1.625,1,0),(['reverse']*3,'reverse',1.125,0,0),(['forward','reverse'],'forward',1.175,0,1)])
def test_objective_keeps_initial_and_internal_changes_separate(directions,initial,total,initial_change,inside):
 # Break: omit initial fee, count terminal as action, or double-count first switch.
 s=m.objective_summary(toy_plan(directions),request(),initial)
 assert s['nominal_objective_m']==pytest.approx(total)
 assert s['initial_direction_changes']==initial_change and s['internal_direction_changes']==inside

@pytest.mark.parametrize('goal',[(.6,0.),(-.6,0.)])
def test_none_reproduces_original_search(goal):
 # Break: altering primitives/priority/defaults when no new direction is supplied.
 r=request(goal);expected=old.plan_kinodynamic_path(old.KinodynamicPlanRequest(**vars(r)),model())
 result=new.plan_kinodynamic_path(r,model(),initial_direction=None)
 assert asdict(result)==asdict(expected)

def test_zero_existing_switch_cost_keeps_legacy_choice():
 # Break: restricting roots despite zero objective penalty.
 r=request(switch=0.);legacy=old.plan_kinodynamic_path(old.KinodynamicPlanRequest(**vars(r)),model())
 result=new.plan_kinodynamic_path(r,model(),initial_direction='forward')
 assert asdict(result)==asdict(legacy)

def test_current_forward_still_allows_required_reverse():
 # Break: forcing forward instead of charging nominal root fee.
 r=request();result=new.plan_kinodynamic_path(r,model(),initial_direction='forward')
 assert result.status=='success' and result.waypoints[0].main_command_rad_s<0
 assert np.linalg.norm(np.array([result.waypoints[-1].x_m,result.waypoints[-1].y_m])-[-.6,0.])<=.2+1e-12
 assert m.objective_summary(result,r,'forward')['initial_direction_changes']==1

@pytest.mark.parametrize('direction',['',0,np.nan,'backward'])
def test_bad_direction_rejected_before_search(direction):
 # Break: invalid direction defaults silently or bypasses validation at invalid start.
 r=replace(request(),occupancy_grid=np.ones((61,61),np.uint8))
 with pytest.raises(ValueError,match='initial_direction'):new.plan_kinodynamic_path(r,model(),initial_direction=direction)

def snapshot_inputs():
 labels=np.full((100,100),-1,np.int8);times=np.full(labels.shape,-np.inf)
 raw=dict(labels=labels,provenance=np.zeros_like(labels),floor_time=times.copy(),hazard_time=times.copy(),origin=np.array([-5.,-5.]),resolution=.1,time_s=6.)
 cur=dict(frame=60,time_s=6.,position_world_xy=np.zeros(2),yaw_world_rad=0.,pointgoal_base_xy=np.array([4.,0.]),accepted_direction='forward')
 return raw,cur

def test_snapshot_copies_current_map_and_goal():
 # Break: caller aliases can rewrite future prepared problem or fabricate observed free.
 raw,cur=snapshot_inputs();s=m.snapshot_problem(raw,cur);raw['labels'][:]=0;cur['pointgoal_base_xy'][:]=-3
 assert np.all(s['map']['labels']==-1) and not s['map']['labels'].flags.writeable
 np.testing.assert_equal(s['current']['pointgoal_base_xy'],[4.,0.])
 r,meta=m.prepare_request(s)
 assert r.node_budget==3000 and meta['unknown_fraction']==1. and not meta['certified_safe']
 assert np.linalg.norm(np.asarray(r.goal_xy)-[2.4,0.])<1e-12
 assert np.all(s['map']['labels']==-1)

def test_snapshot_rejects_future_payload_without_reading_it():
 # Break: unrestricted dict forwarding reads a future actual plan/labels.
 raw,cur=snapshot_inputs()
 class Trap(dict):
  def __getitem__(self,key):
   if key=='future_applied':raise AssertionError('future payload was read')
   return super().__getitem__(key)
 with pytest.raises(ValueError,match='current fields'):m.snapshot_problem(raw,Trap(dict(cur,future_applied=None)))

@pytest.mark.parametrize('bad',['map_clock','evidence_clock','zero_resolution'])
def test_snapshot_rejects_noncurrent_geometry_before_search(bad):
 # Break: applying future map, future evidence or invalid metric scale.
 raw,cur=snapshot_inputs()
 if bad=='map_clock':raw['time_s']=6.1
 elif bad=='evidence_clock':raw['labels'][0,0]=1;raw['hazard_time'][0,0]=6.1
 else:raw['resolution']=0.
 with pytest.raises(ValueError):m.snapshot_problem(raw,cur)
