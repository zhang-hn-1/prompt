"""Read composed robot schema only; no simulation/extra dynamic sequence."""
import json,hashlib,sys
from pathlib import Path
sys.path.insert(0,'/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
from pxr import Usd
H=Path(__file__).resolve().parent;P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
asset=P.parents[1]/'project/rl_env_0.4/rl_env/rotunbot_wheel_sim_model_0.4/Rotunbot_wheel.usd'
s=Usd.Stage.Open(str(asset));joints=[];bodies=[]
for p in s.Traverse():
    if 'Joint' in str(p.GetTypeName()):
        joints.append(dict(path=str(p.GetPath()),type=str(p.GetTypeName()),axis=str(p.GetAttribute('physics:axis').Get()),body0=[str(v) for v in p.GetRelationship('physics:body0').GetTargets()],body1=[str(v) for v in p.GetRelationship('physics:body1').GetTargets()]))
    if p.HasAttribute('physics:mass'):
        bodies.append(dict(path=str(p.GetPath()),mass_kg=p.GetAttribute('physics:mass').Get(),center_of_mass=str(p.GetAttribute('physics:centerOfMass').Get()),diagonal_inertia=str(p.GetAttribute('physics:diagonalInertia').Get())))
files=[Path(layer.realPath) for layer in s.GetUsedLayers() if layer.realPath];runtime=P/'benchmark/isaac_runtime.py'
result=dict(scope='composedUSDstaticjoint/body schema; not physicsDOF-order readback or intervention',robot_asset=str(asset),joints=joints,bodies=bodies,
    confirmed_runtime_command_indices=[0,1],third_recorded_joint_is_not_commanded_by_execute=True,
    index_to_joint_name_mapping='not asserted: new runner did not save runtime dof_names; composed USD traversal order does not certify articulation DOF order',
    warning='cannot delete third state as irrelevant merely because uncommanded; its actual motion differs. PID carryover and commanded targets also differ independently.',
    source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files+[runtime]})
(H/'short_robot_joint_schema.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
