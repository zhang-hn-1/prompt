"""Actual runtime initialization/render check; no navigation score or control change."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent
PROJECT=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(PROJECT))
import scripts.run_pointgoal_benchmark as runner
from scripts.camera_rig import camera_rig_metadata
sys.path.insert(0,str(HERE.parent/'bilateral_temporal_geometry_navigation_20261003_v1'))
from render_boundary import hide_diagnostic_markers
sys.path.insert(0,str(HERE.parent/'clean_visual_hard0_intervention_20261003_v1'))
from clock_sync import ensure_current_frames

class CaptureRuntime(runner.IsaacBenchmarkRuntime):
    @staticmethod
    def _create_markers(stage,episode,path,gf,usd_geom):
        result=runner.IsaacBenchmarkRuntime._create_markers(stage,episode,path,gf,usd_geom)
        hide_diagnostic_markers(stage,usd_geom)
        return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--episode',required=True);args=parser.parse_args()
    episode=next(e for e in runner.load_manifest(PROJECT/'configs/benchmark_v1/golden_v1_manifest.json').episodes if e.episode_id==args.episode)
    dest=HERE/'render'/args.episode;dest.mkdir(parents=True,exist_ok=True)
    occupancy=runner._resolve_occupancy_path(PROJECT/'configs/benchmark_v1/scene_index.json',episode,PROJECT/'data')
    runtime=CaptureRuntime(runner.IsaacRuntimeConfig(scene_root=Path('/home/an/project/NavDP/assets/scenes'),
        contract_path=PROJECT/'configs/data_contract.yaml',occupancy_path=occupancy,gui=False,
        record_video=False,lighting_profile='fixed_camera_fill_v1',collision_mode='physical_contact_v2'))
    try:
        runtime.reset(episode)
        import omni.usd
        from pxr import UsdGeom
        from collect_data_dubins import LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH,_camera_optical_to_base,_get_rgb
        from bev_ground_truth import read_camera_depth
        stage=omni.usd.get_context().get_stage();clock=runtime._clock_metadata()
        before=[np.array(v) for v in (*runtime._robot.get_world_pose(),runtime._robot.get_joint_positions(),runtime._robot.get_joint_velocities())]
        for camera in (runtime._left_camera,runtime._right_camera):camera.set_frequency(-1)
        frames=ensure_current_frames((runtime._left_camera,runtime._right_camera),clock[2],runtime._world.render)
        assert clock==runtime._clock_metadata()
        after=[np.array(v) for v in (*runtime._robot.get_world_pose(),runtime._robot.get_joint_positions(),runtime._robot.get_joint_velocities())]
        assert all(np.array_equal(a,b) for a,b in zip(before,after))
        measured={};arrays={}
        for eye,path,camera in [('left',LEFT_CAMERA_PATH,runtime._left_camera),('right',RIGHT_CAMERA_PATH,runtime._right_camera)]:
            xform=UsdGeom.Xformable(stage.GetPrimAtPath(path));local=xform.GetLocalTransformation()
            translation=np.array(local.ExtractTranslation());expected=np.array([.2,.3 if eye=='left' else -.3,.05])
            np.testing.assert_allclose(translation,expected,atol=1e-6,rtol=0)
            extrinsic=_camera_optical_to_base(stage,path)
            np.testing.assert_allclose(extrinsic[:3,3],expected,atol=1e-6)
            np.testing.assert_allclose(extrinsic[:3,:3],[[0,0,1],[0,-1,0],[1,0,0]],atol=1e-6)
            rgb=_get_rgb(camera);depth=read_camera_depth(camera)
            Image.fromarray(rgb).save(dest/f'{eye}_rgb.png');arrays[f'{eye}_depth_m']=depth
            measured[eye]=dict(prim_path=path,local_translation_m=translation.tolist(),optical_to_base_link=extrinsic.tolist(),
                rgb_shape=list(rgb.shape),valid_depth_fraction=float(np.mean(np.isfinite(depth)&(depth>0))),
                rendering_time_s=float(camera.get_current_frame()['rendering_time']),raw_physics_time_s=float(clock[2]))
        np.savez_compressed(dest/'depth.npz',**arrays)
        (dest/'measurement.json').write_text(json.dumps(dict(episode=args.episode,nominal_rig=camera_rig_metadata(),actual=measured,
            render_preserved_physics=True,lighting_profile=runtime.config.lighting_profile,
            scope='reset and current-frame rendering only; no navigation outcome'),indent=2)+'\n')
        if not (HERE/'base_link_masks.npz').exists():
            sys.path.insert(0,str(HERE.parent/'fixed_rig_self_mask_and_curve_qualification_20261003_v1'))
            import build_current_mask
            build_current_mask.main()
        print('CAMERA_CHECK_PASSED',args.episode,json.dumps({eye:v['local_translation_m'] for eye,v in measured.items()}),flush=True)
    except BaseException:
        import traceback
        error=traceback.format_exc()
        (dest/'failure.txt').write_text(error)
        print(error,flush=True)
        runtime.close(return_code=1)
        raise
    finally:
        runtime.close()

if __name__=='__main__':main()
