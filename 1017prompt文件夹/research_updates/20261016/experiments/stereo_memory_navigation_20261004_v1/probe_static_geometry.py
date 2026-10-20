"""One static live measurement of actual Camera intrinsics; no motion/model input."""
import sys,json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
from scripts.run_tracking_acceptance import DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,DEFAULT_FIXTURE,DEFAULT_OCCUPANCY,DEFAULT_SCENE_ROOT,build_tracking_episode
from scripts.run_round4_fixed_reference import load_transformed_reference
from benchmark.isaac_runtime import IsaacBenchmarkRuntime,IsaacRuntimeConfig
ref,_=load_transformed_reference(H.parent/'oracle_fixed_reference_pacing_20260928_v1/0002_reference.npz',case_id='0002')
episode=build_tracking_episode(ref.case,ref,scene_root=DEFAULT_SCENE_ROOT,fixture_path=DEFAULT_FIXTURE,occupancy_path=DEFAULT_OCCUPANCY,seed=3072)
runtime=IsaacBenchmarkRuntime(IsaacRuntimeConfig(scene_root=DEFAULT_SCENE_ROOT.resolve(),contract_path=DEFAULT_CONTRACT.resolve(),gui=False,
    record_video=False,occupancy_path=DEFAULT_OCCUPANCY.resolve(),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),diagnostic_open_plane=True))
runtime.set_reference_visualization_path(ref.points_xy)
code=1
try:
    runtime.reset(episode)
    from pxr import UsdGeom
    import omni.usd
    stage=omni.usd.get_context().get_stage();results={}
    for eye,camera in [('left',runtime._left_camera),('right',runtime._right_camera)]:
        prim=UsdGeom.Camera(stage.GetPrimAtPath(camera.prim_path))
        results[eye]=dict(intrinsics=np.asarray(camera.get_intrinsics_matrix()).tolist(),resolution=list(camera.get_resolution()),
            focal_length=float(prim.GetFocalLengthAttr().Get()),horizontal_aperture=float(prim.GetHorizontalApertureAttr().Get()),vertical_aperture=float(prim.GetVerticalApertureAttr().Get()))
    from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,get_cam_pose,_get_rgb
    from bev_ground_truth import read_camera_depth
    frames=H/'static_geometry';frames.mkdir(exist_ok=True)
    # Six renders of unmodified scene at exactly the same physics time.
    clocks=[]
    for camera in (runtime._left_camera,runtime._right_camera):camera.set_frequency(-1)
    for i in range(6):
        before=runtime._clock_metadata();runtime._world.render();assert runtime._clock_metadata()==before
        np.savez_compressed(frames/f'floor_{i}.npz',left_depth=read_camera_depth(runtime._left_camera),right_depth=read_camera_depth(runtime._right_camera),left_rgb=_get_rgb(runtime._left_camera),right_rgb=_get_rgb(runtime._right_camera),left_pose=get_cam_pose(stage,LEFT_CAMERA_PATH),right_pose=get_cam_pose(stage,RIGHT_CAMERA_PATH))
        clocks.append(dict(index=i,physics_clock=list(before),left_render_time=float(runtime._left_camera.get_current_frame()['rendering_time'])))
    ground=[]
    for prim in stage.Traverse():
        if 'Ground' in str(prim.GetPath()) and prim.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(prim);points=np.asarray(mesh.GetPointsAttr().Get());transform=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
            world=np.c_[points,np.ones(len(points))]@transform.T
            ground.append(dict(path=str(prim.GetPath()),z_range=[float(world[:,2].min()),float(world[:,2].max())]))
    # Known world-x plane, non-repeating bright squares, no collision and no physics action.
    from pxr import Gf,Sdf
    poses=[get_cam_pose(stage,p) for p in (LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH)]
    center=(poses[0][:3,3]+poses[1][:3,3])/2
    plane_x=float(center[0]+2.)
    mesh=UsdGeom.Mesh.Define(stage,'/World/CalibrationOnlyBoard')
    points=[];faces=[];colors=[]
    rng=np.random.default_rng(4242)
    for i in range(12):
      for j in range(12):
        y0=center[1]-2+i/3;z0=center[2]-2+j/3;k=len(points)
        points.extend([(plane_x,y0,z0),(plane_x,y0+1/3,z0),(plane_x,y0+1/3,z0+1/3),(plane_x,y0,z0+1/3)])
        faces.extend([k,k+1,k+2,k+3]);colors.append(tuple(rng.uniform(.25,1,3)))
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points]);mesh.CreateFaceVertexCountsAttr([4]*144);mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateDoubleSidedAttr(True)
    display=mesh.CreateDisplayColorAttr([Gf.Vec3f(*c) for c in colors]);UsdGeom.Primvar(display).SetInterpolation('uniform')
    for i in range(6):
        before=runtime._clock_metadata();runtime._world.render();assert runtime._clock_metadata()==before
        np.savez_compressed(frames/f'board_{i}.npz',left_depth=read_camera_depth(runtime._left_camera),right_depth=read_camera_depth(runtime._right_camera),left_rgb=_get_rgb(runtime._left_camera),right_rgb=_get_rgb(runtime._right_camera),left_pose=get_cam_pose(stage,LEFT_CAMERA_PATH),right_pose=get_cam_pose(stage,RIGHT_CAMERA_PATH))
    (H/'static_geometry_identity.json').write_text(json.dumps(dict(intrinsics=results,plane_x=plane_x,ground_meshes=ground,clocks=clocks,role='static interface audit, no movement/navigation/scoring'),indent=2)+'\n');code=0
finally:runtime.close(return_code=code)
