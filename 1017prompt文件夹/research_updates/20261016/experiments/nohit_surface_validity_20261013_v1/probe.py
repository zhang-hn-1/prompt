from pathlib import Path
import json,numpy as np
D=Path(__file__).resolve().parent
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'anti_aliasing':0,'width':160,'height':120})
try:
 from isaacsim.core.api import World
 from isaacsim.sensors.camera import Camera
 from pxr import UsdGeom,UsdLux,Gf
 import omni.usd
 world=World(stage_units_in_meters=1.);stage=omni.usd.get_context().get_stage();light=UsdLux.DomeLight.Define(stage,'/World/ProbeLight');light.CreateIntensityAttr(1000.)
 cube=UsdGeom.Cube.Define(stage,'/World/ProbeBox');cube.CreateSizeAttr(.5);trans=cube.AddTranslateOp();trans.Set(Gf.Vec3d(0,0,-2));cube.CreateDisplayColorAttr([Gf.Vec3f(.2,.4,.8)])
 cam=Camera(prim_path='/World/ProbeCamera',resolution=(160,120));world.reset();cam.initialize();cam.set_world_pose(position=np.zeros(3),orientation=np.array([1.,0,0,0]),camera_axes='usd');cam.set_clipping_range(.1,10.);cam.add_distance_to_image_plane_to_frame();rows=[]
 for label,z in [('cube2m',-2.),('clipped15m',-15.)]:
  trans.Set(Gf.Vec3d(0,0,z))
  for _ in range(30):world.step(render=True)
  f=cam.get_current_frame();depth=np.array(f['distance_to_image_plane'],copy=True);rgb=np.array(f['rgba'],copy=True);assert depth.shape==(120,160);np.savez_compressed(D/f'{label}.npz',depth_m=depth,rgb=rgb,camera_usd_world=np.array(UsdGeom.Xformable(cam.prim).ComputeLocalToWorldTransform(0)),clipping_range=np.array(cam.get_clipping_range()))
  centre=float(depth[60,80]);row=dict(label=label,centre_depth=centre if np.isfinite(centre) else str(centre),positive_inf=int(np.isposinf(depth).sum()),zero=int((depth==0).sum()),nan=int(np.isnan(depth).sum()),finite_positive=int((np.isfinite(depth)&(depth>0)).sum()),pixels=depth.size);rows.append(row);print(row,flush=True)
 passed=abs(rows[0]['centre_depth']-1.75)<.03 and rows[0]['positive_inf']+rows[0]['zero']>0 and rows[1]['finite_positive']==0 and all(r['nan']==0 for r in rows)
 (D/'semantic_result.json').write_text(json.dumps(dict(rows=rows,gate=bool(passed),no_hit_output='positive_inf' if rows[1]['positive_inf']==19200 else 'other',probe_physical_run=1,not_navigation=True,fit=0),indent=2)+'\n')
finally:app.close()
