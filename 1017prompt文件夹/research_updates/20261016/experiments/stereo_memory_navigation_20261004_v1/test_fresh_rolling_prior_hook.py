"""Exercise actual derived physics hook without initializing Isaac/GPU."""
import ast
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from rolling_shadow import OnlineRollingShadow,emulate_sensor_row

def make_runtime(rolling):
 source=ast.parse((Path(__file__).parent/'run_fresh_rolling_prior_shadow.py').read_text())
 cls=next(n for n in ast.walk(source) if isinstance(n,ast.ClassDef) and n.name=='CaptureRuntime')
 hook=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_record_physics_diagnostic')
 stripped=ast.ClassDef(name='CaptureRuntime',bases=cls.bases,keywords=[],body=[hook],decorator_list=[])
 class Base:
  def __init__(self):self._physics_diagnostics={k:[] for k in ['simulation_time_s','root_quaternion_world_wxyz','angular_velocity_world_rad_s','joint_velocity_rad_s']};self.request=np.array([.3,.2])
  def _clock_metadata(self):return (len(self._physics_diagnostics['simulation_time_s']),)
  def _record_physics_diagnostic(self,t):
   row=dict(simulation_time_s=t,root_quaternion_world_wxyz=[1.,0.,0.,0.],angular_velocity_world_rad_s=[0.,0.,0.],joint_velocity_rad_s=[.5,0.,0.])
   for k,v in row.items():self._physics_diagnostics[k].append(v)
 ns=dict(runner=SimpleNamespace(IsaacBenchmarkRuntime=Base),np=np,rolling=rolling,emulate_sensor_row=emulate_sensor_row)
 exec(compile(ast.fix_missing_locations(ast.Module(body=[stripped],type_ignores=[])),'actual_hook','exec'),ns)
 return ns['CaptureRuntime']()

def test_actual_hook_live_each_step_and_does_not_change_commands_or_parent_state():
 rolling=OnlineRollingShadow(.4);runtime=make_runtime(rolling)
 for i in range(11):runtime._record_physics_diagnostic(i*.02)
 assert len(rolling.records)==11
 np.testing.assert_allclose(rolling.records[-1]['position'],[.04,0,0])
 np.testing.assert_array_equal(runtime.request,[.3,.2])
 assert runtime._clock_metadata()==(11,)
 assert rolling.sensors[-1]['time_s']==.2

def test_actual_hook_prefix_prediction_unchanged_by_future_suffix():
 a=OnlineRollingShadow(.4);b=OnlineRollingShadow(.4);ra=make_runtime(a);rb=make_runtime(b)
 for i in range(6):ra._record_physics_diagnostic(i*.02);rb._record_physics_diagnostic(i*.02)
 before=[dict(r) for r in a.records]
 for i in range(6,12):rb._record_physics_diagnostic(i*.02)
 assert a.records==before==b.records[:6]
