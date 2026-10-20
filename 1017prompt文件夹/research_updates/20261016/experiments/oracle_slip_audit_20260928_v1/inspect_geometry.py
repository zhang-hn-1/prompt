from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from pxr import Usd,UsdPhysics,UsdGeom
import json
from pathlib import Path
p=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/project/rl_env_0.4/rl_env/rotunbot_wheel_sim_model_0.4/Rotunbot_wheel.usd')
s=Usd.Stage.Open(str(p));out=[]
for prim in s.Traverse():
 if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI) or 'Joint' in prim.GetTypeName() or 'Material' in prim.GetTypeName():
  attrs={a.GetName():str(a.Get()) for a in prim.GetAttributes() if any(k in a.GetName().lower() for k in ['physics','radius','extent','xform','approximation'])};rels={r.GetName():[str(t) for t in r.GetTargets()] for r in prim.GetRelationships()};out.append({'path':str(prim.GetPath()),'type':prim.GetTypeName(),'rigid':prim.HasAPI(UsdPhysics.RigidBodyAPI),'attributes':attrs,'relationships':rels})
Path(__file__).with_name('asset_recheck.json').write_text(json.dumps(out,indent=2));cache=UsdGeom.XformCache();sphere=s.GetPrimAtPath('/Rotunbot_wheel/link1/collisions/mesh_0/sphere');link=s.GetPrimAtPath('/Rotunbot_wheel/link1');mat=cache.GetLocalToWorldTransform(sphere)*cache.GetLocalToWorldTransform(link).GetInverse();Path(__file__).with_name('sphere_transform.json').write_text(json.dumps({'sphere_to_link':[[float(x) for x in row] for row in mat]},indent=2));app.close()
