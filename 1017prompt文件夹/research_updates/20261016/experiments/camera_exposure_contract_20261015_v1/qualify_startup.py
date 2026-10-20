"""Execute the actual startup override against deterministic module/lifecycle stubs."""
from pathlib import Path
import ast,sys,types,json
D=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');assert (P/'scripts/collect_data_dubins.py').is_file();tree=ast.parse((D.parent/'wide_pinhole_camera_20261015_v1/native_probe.py').read_text());cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='Runtime');fn=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_initialize');events=[]
app=types.ModuleType('isaacsim');sensor=types.ModuleType('omni.isaac.sensor');c=types.ModuleType('collect_data_dubins');c.LEFT_CAMERA_PATH='left';c.RIGHT_CAMERA_PATH='right';originalCamera=object;sensor.Camera=originalCamera
class App:
 def __init__(self,*a,**k):events.append('SimulationApp_started')
app.SimulationApp=App
class Base:
 def _initialize(self,*a,**k):
  self._simulation_app=app.SimulationApp({});assert str(P/'scripts') in sys.path;events.append('Original_camera_initialization_after_hook');assert sensor.Camera is not originalCamera

def hook(module,specs):
 assert events[-1]=='SimulationApp_started';assert str(P/'scripts') in sys.path;events.append('SDK_hook_after_app_and_path_before_camera');module.Camera=type('Hooked',(),{});return originalCamera
saved={k:sys.modules.get(k) for k in ['isaacsim','omni','omni.isaac','omni.isaac.sensor','collect_data_dubins']};omni=types.ModuleType('omni');omni.__path__=[];isaac=types.ModuleType('omni.isaac');isaac.__path__=[];omni.isaac=isaac;isaac.sensor=sensor;sys.modules.update({'isaacsim':app,'omni':omni,'omni.isaac':isaac,'omni.isaac.sensor':sensor,'collect_data_dubins':c});saved_path=list(sys.path)
try:
 node=ast.ClassDef(name='Runtime',bases=[ast.Name(id='IsaacBenchmarkRuntime',ctx=ast.Load())],keywords=[],body=[fn],decorator_list=[]);mod=ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[]));ns=dict(IsaacBenchmarkRuntime=Base,P=P,sys=sys,install_preinitialized_camera=hook,cfg=dict(focal_mm=16.45/3,positions_m=[[.45,0,.05],[-.45,0,.05]],yaw_degrees=[0,180]));exec(compile(mod,'actual_startup_override','exec'),ns);obj=ns['Runtime']();obj._initialize(None,None,None);assert app.SimulationApp is App and sensor.Camera is originalCamera;assert isinstance(obj._simulation_app,App)
finally:
 sys.path[:]=saved_path
 for k,v in saved.items():
  if v is None:sys.modules.pop(k,None)
  else:sys.modules[k]=v
(D/'startup_qualification.json').write_text(json.dumps(dict(pass_check=True,actual_override_AST_executed=True,events=events,original_app_camera_restored=True,app_owned_before_hook_failure_cleanup=True,real_collect_script_exists=True,scope='Deterministic startup contract, no real SDK exposure qualification or camera success claim.'),indent=2)+'\n');print(events)
