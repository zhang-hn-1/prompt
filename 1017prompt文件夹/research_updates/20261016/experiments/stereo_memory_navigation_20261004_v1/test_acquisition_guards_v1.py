"""Execute the driver's actual guard/outer cleanup AST with controlled doubles."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
P=Path(__file__).with_name('run_hypothesis_learning_acquisition_v1.py')
TREE=ast.parse(P.read_text())
MAIN=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='main')
ACT=next(n for n in ast.walk(MAIN) if isinstance(n,ast.FunctionDef) and n.name=='recorded_act')
class PassedZeroGuard(Exception):pass

def invoke(action):
 policy=NS(grid=NS(birth=True),set_source_packet=lambda x:None,set_current_packet=lambda x:None)
 env=dict(np=np,time=NS(perf_counter=lambda:0.),policies=[],pose_bridge=NS(at=lambda t:{}),current_packet={'time_s':0.},navigation_observation=NS(build=lambda *a:(NS(position_world_xy=np.zeros(2),proprio_history=np.zeros((1,10))),None)),rolling=NS(sensors=[{'gyro_body_rad_s':np.zeros(3)}]),episode=NS(goal_position_world_m=[0,0,0]),policy=policy,partial_source=NS(birth_packet=None),source_packet={},act=lambda *a:np.asarray(action),gate=NS(choose=lambda *a:np.asarray(action)),source_event=None,require_bootstrap_progress=lambda *a,**k:(_ for _ in ()).throw(PassedZeroGuard()))
 # The policy-record time is read by the actual gate call.
 policy.records=[{'time_s':0.}]
 exec(compile(ast.Module(body=[ACT],type_ignores=[]),str(P),'exec'),env)
 env['recorded_act'](None,None)

def test_nonzero_fails_before_execute_or_shadow():
 with pytest.raises(AssertionError,match='BEFORE'):invoke(np.ones(10))
def test_zero_passes_execution_guard():
 with pytest.raises(PassedZeroGuard):invoke(np.zeros(10))
def test_setup_failure_closes_runtime():
 outer=next(n for n in MAIN.body if isinstance(n,ast.Try) and n.finalbody and any(isinstance(a,ast.Expr) and isinstance(a.value,ast.Call) and isinstance(a.value.func,ast.Attribute) and a.value.func.attr=='close' for a in n.finalbody))
 calls=[]
 def fail(*a):raise RuntimeError('controlled setup failure')
 env=dict(runtime=NS(close=lambda **k:calls.append(k)),runner=NS(load_action_protocol=fail),P=Path('/tmp'),code=1)
 with pytest.raises(RuntimeError,match='setup failure'):exec(compile(ast.Module(body=[outer],type_ignores=[]),str(P),'exec'),env)
 assert calls==[{'return_code':1}]
