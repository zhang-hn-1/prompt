from pathlib import Path
import sys,json,hashlib,numpy as np
from scipy.spatial.transform import Rotation
D=Path(__file__).resolve().parent;W=D.parents[1];H=W/'experiments/stereo_memory_navigation_20261004_v1';P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(H),str(P)]
from run_original_E1_direct_memory_navigation_v1 import IsaacBenchmarkRuntime,IsaacRuntimeConfig,DEFAULT_CONTRACT,DEFAULT_DRIVE_PROFILE,hide_diagnostic_markers,install_sigterm_cleanup,export_then_close,write_rgb_video
from benchmark.manifest import BenchmarkEpisode
cfg=json.loads((D/'protocol.json').read_text());assert json.loads((D/'cad_qualification.json').read_text())['pass_gate'];out=D/'render_recovery_run';out.mkdir(exist_ok=False);(out/'frames').mkdir();R=Path(cfg['record_root']);phys=np.load(R/'physics.npz');sourcehash={}
class Probe(IsaacBenchmarkRuntime):
 @staticmethod
 def _create_markers(stage,ep,path,gf,usd_geom):
  r=IsaacBenchmarkRuntime._create_markers(stage,ep,path,gf,usd_geom);hide_diagnostic_markers(stage,usd_geom);return r
 def frozen_state(self):return [np.asarray(v).copy() for v in (*self._robot.get_world_pose(),self._robot.get_linear_velocity(),self._robot.get_angular_velocity(),self._robot.get_joint_positions(),self._robot.get_joint_velocities())]
install_sigterm_cleanup();runtime=Probe(IsaacRuntimeConfig(scene_root=Path(cfg['scene_root']),contract_path=DEFAULT_CONTRACT.resolve(),occupancy_path=Path(cfg['occupancy']),drive_profile_path=DEFAULT_DRIVE_PROFILE.resolve(),gui=False,record_video=False,lighting_profile=cfg['lighting_profile'],collision_mode='physical_contact_v2',diagnostic_open_plane=False,diagnostic_physics_trace=True,auxiliary_velocity_limit_rad_s=.3));code=1;rows=[];video=[]
try:
 ep=BenchmarkEpisode.from_dict(next(e for e in json.loads(Path(cfg['episode_manifest']).read_text())['episodes'] if e['episode_id']=='golden_e1_0001'));runtime.reset(ep)
 import omni.usd
 from pxr import UsdGeom,Gf
 from collect_data_dubins import _get_rgb,get_cam_pose,LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH
 from bev_ground_truth import read_camera_depth
 stage=omni.usd.get_context().get_stage();cameras=[runtime._left_camera,runtime._right_camera];paths=[LEFT_CAMERA_PATH,RIGHT_CAMERA_PATH];ROT=np.array([[0,0,1],[0,-1,0],[1,0,0.]]);AX=np.diag([1,-1,-1]);clock=runtime._clock_metadata()
 for c in cameras:c.set_frequency(-1)
 for j,fp in enumerate(sorted((R/'frames').glob('frame_*.npz'))):
  a=np.load(fp);t=float(a['time_s']);idx=int(np.argmin(abs(phys['simulation_time_s']-t)));q=a['root_quaternion_world_wxyz'];root=a['root_position_world_m'];runtime._robot.set_world_pose(position=root.astype(np.float32),orientation=q.astype(np.float32));runtime._robot.set_joint_positions(phys['joint_position_rad'][idx]);runtime._robot.set_joint_velocities(np.zeros(3));runtime._robot.set_linear_velocity(np.zeros(3));runtime._robot.set_angular_velocity(np.zeros(3));runtime._world.render();assert runtime._clock_metadata()==clock,'initial sync advanced physics clock';assert np.allclose(runtime._robot.get_world_pose()[0],root,atol=2e-5,rtol=0),'initial pose sync';captured={};expected={}
  for rig in ['baseline','candidate']:
   positions=cfg['baseline']['positions_m'] if rig=='baseline' else [cfg['layout']['front_position_m'],cfg['layout']['rear_position_m']];degrees=cfg['baseline']['yaw_degrees'] if rig=='baseline' else cfg['layout']['yaw_degrees']
   for k,path in enumerate(paths):
    T=np.eye(4);T[:3,3]=positions[k];T[:3,:3]=Rotation.from_euler('z',degrees[k],degrees=True).as_matrix()@ROT@AX;x=UsdGeom.Xformable(stage.GetPrimAtPath(path));x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d(*T.T.flatten().tolist()));expected[rig+'_'+str(k)]=T
   state=runtime.frozen_state()
   for unused in range(4):
    runtime._world.render();assert runtime._clock_metadata()==clock;after=runtime.frozen_state();assert all(np.array_equal(x,y) for x,y in zip(state,after)),'render changed physical state'
   actualroot,actualq=runtime._robot.get_world_pose();base=np.eye(4);base[:3,:3]=Rotation.from_quat(np.r_[actualq[1:],actualq[0]]).as_matrix();base[:3,3]=actualroot
   for k,(camera,path) in enumerate(zip(cameras,paths)):
    actual=get_cam_pose(stage,path);T=base@expected[rig+'_'+str(k)];assert np.allclose(actual,T,atol=2e-5,rtol=0),(rig,j,'camera pose readback',np.max(abs(actual-T)));rgb=_get_rgb(camera).copy();depth=np.asarray(read_camera_depth(camera)).copy();assert rgb.shape==(480,640,3) and depth.shape==(480,640);captured[rig+'_'+str(k)+'_rgb']=rgb;captured[rig+'_'+str(k)+'_gt']=depth;captured[rig+'_'+str(k)+'_usd']=actual
  assert np.allclose(runtime._robot.get_joint_positions(),phys['joint_position_rad'][idx],atol=2e-5,rtol=0)
  captured.update(source_time_s=t,root_position=root,root_quaternion=q,joints=phys['joint_position_rad'][idx],renderer_clock=np.array(clock[:2]));dst=out/'frames'/fp.name;np.savez_compressed(dst,**captured);sourcehash[str(fp)]=hashlib.sha256(fp.read_bytes()).hexdigest();rows.append(dict(frame=fp.name,source_time_s=t,physics_index=idx,clock=clock,sha256=hashlib.sha256(dst.read_bytes()).hexdigest()))
  video.append(np.hstack([captured['baseline_0_rgb'],captured['baseline_1_rgb'],captured['candidate_0_rgb'],captured['candidate_1_rgb']]))
  if j%30==0:print('complete poses',j+1,'/181',flush=True)
 assert len(rows)==181;write_rgb_video(out/'baseline_front_rear.mp4',video,fps=2.);(out/'capture_seal.json').write_text(json.dumps(dict(complete=True,rows=rows,inputs=sourcehash,physics_source_sha256=hashlib.sha256((R/'physics.npz').read_bytes()).hexdigest(),clock_frozen=True,new_navigation=False,model_inference=0,GT_offline_only=True,same_runtime_lighting=True),indent=2)+'\n');code=0
except BaseException:
 import traceback
 (out/'exception.txt').write_text(traceback.format_exc());raise
finally:
 def export():
  (out/'terminal.json').write_text(json.dumps(dict(exit_code=code,completed_frames=len(rows),clock=runtime._clock_metadata(),not_navigation=True))+'\n')
 export_then_close(runtime,export,code)
