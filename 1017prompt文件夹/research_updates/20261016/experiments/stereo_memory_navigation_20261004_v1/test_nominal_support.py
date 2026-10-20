"""Expose missing cap, wrong direction, original-model mutation, bad inputs."""
from pathlib import Path
import os,sys,json,importlib,copy,math
import pytest
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path.insert(0,str(P))
from benchmark.drive_model import DriveModel
from bank_mapping import BankMap
m=importlib.import_module(os.environ.get('NOMINAL_SUPPORT_MODULE','nominal_support'))
H=Path(__file__).resolve().parent

def models():
 base=DriveModel.from_json(P/'configs/benchmark_v1/drive_dynamics_v1.json')
 bm=BankMap(json.loads((H.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json').read_text()))
 return base,bm,m.make_support_drive(base,bm)

@pytest.mark.parametrize('main,aux,want',[(.5,-math.pi/6,.781502366065979),(.5,math.pi/6,-.8272253274917603),(-.5,-math.pi/6,.7672796845436096),(-.5,math.pi/6,-.7978797554969788)])
def test_extreme_nominal_does_not_request_outside_consumer_domain(main,aux,want):
 base,bm,cap=models();assert cap.curvature(main,aux)==pytest.approx(want,abs=1e-12)
 for row in bm.directions['forward' if main>0 else 'reverse'].values():assert not bm.query(main,row['speed_m_s'],cap.curvature(main,aux))['curvature_clamped']

def test_feasible_curvature_unchanged():
 base,bm,cap=models()
 for main in [.5,-.5,1.,-1.,1.5,-1.5]:assert cap.curvature(main,0)==base.curvature(main,0)

def test_planner_subclass_preserves_speed_and_original_response():
 base,bm,cap=models();old=base.curvature(.5,-math.pi/6)
 assert isinstance(cap,DriveModel) and cap is not base
 for main in [.5,-.5,1.,-1.,1.5,-1.5]:
  for aux in [-math.pi/6,0,math.pi/6]:assert cap.speed(main,aux)==base.speed(main,aux)
 assert base.curvature(.5,-math.pi/6)==old and old==pytest.approx(.8800519662098094)

@pytest.mark.parametrize('main,aux',[(0.,0.),(float('nan'),0.),(.5,float('inf'))])
def test_invalid_command_not_admitted(main,aux):
 base,bm,cap=models()
 with pytest.raises(ValueError):cap.curvature(main,aux)

def test_all_actual_primitive_commands_map_without_curvature_clamp():
 base,bm,cap=models()
 for main in [.5,-.5,1.,-1.,1.5,-1.5]:
  for degree in [0,-10,10,-20,20,-30,30]:
   k=cap.curvature(main,math.radians(degree))
   for speed in [0.,.2,.4,.6]:assert not bm.query(main,speed,k)['curvature_clamped']

def test_disjoint_calibration_rejected():
 base,bm,cap=models();bad=copy.deepcopy(bm);bad.directions['forward']['high']['curvature_body_1_m']=[-3.,-4.,-5.,-6.,-7.]
 with pytest.raises(ValueError):m.make_support_drive(base,bad)
