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
    (H/'live_calibration_probe.json').write_text(json.dumps(results,indent=2)+'\n');print(results,flush=True);code=0
finally:runtime.close(return_code=code)
