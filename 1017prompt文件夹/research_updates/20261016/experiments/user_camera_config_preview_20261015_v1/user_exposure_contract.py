"""Future SDK acquisition only. Renderer pose/depth remain offline labels, never policy input."""
import copy,numpy as np
from scipy.spatial.transform import Rotation
ROT=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
AX=np.diag([1,-1,-1])
def local_transform(position,yaw):
 T=np.eye(4);T[:3,3]=position;T[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@ROT@AX;return T

def install_preinitialized_camera(sensor_module,specs):
 Original=sensor_module.Camera
 class InstalledCamera(Original):
  def __init__(self,*args,**kwargs):
   path=kwargs.get('prim_path',args[0] if args else None);spec=specs.get(path)
   if spec is not None:
    import omni.usd
    from pxr import Gf,UsdGeom
    stage=omni.usd.get_context().get_stage();prim=stage.GetPrimAtPath(path);assert prim and prim.IsA(UsdGeom.Camera)
    T=local_transform(spec['position'],spec['yaw']);x=UsdGeom.Xformable(prim);x.ClearXformOpOrder();x.AddTransformOp().Set(Gf.Matrix4d(*T.T.flatten().tolist()));cam=UsdGeom.Camera(prim);cam.GetFocalLengthAttr().Set(spec['focal_mm']);cam.GetHorizontalApertureAttr().Set(18.);cam.GetVerticalApertureAttr().Set(13.5)
   super().__init__(*args,**kwargs)
   if spec is not None:
    q=Rotation.from_matrix(T[:3,:3]).as_quat();self.set_local_pose(translation=np.asarray(spec['position']),orientation=np.r_[q[3],q[:3]],camera_axes='usd');self.research_local_transform=T.copy()
 sensor_module.Camera=InstalledCamera
 return Original

def attach_camera_parameters(camera):
 import omni.replicator.core as rep
 annotator=rep.AnnotatorRegistry.get_annotator('CameraParams');annotator.attach([camera.get_render_product_path()]);camera._custom_annotators['research_camera_params']=annotator;camera._current_frame['research_camera_params']=None

def snapshot(camera,previous_frame,expected_time,expected_focal_px):
 a=copy.deepcopy(camera.get_current_frame());required=['rendering_frame','rendering_time','rgba','distance_to_image_plane','research_camera_params']
 if any(a.get(k) is None for k in required):raise ValueError('Incomplete atomic camera exposure')
 frame=int(a['rendering_frame']);t=float(a['rendering_time']);rgba=np.asarray(a['rgba']);depth=np.asarray(a['distance_to_image_plane']);p=a['research_camera_params']
 if frame<=previous_frame:raise ValueError('Reused or stale camera exposure')
 if not np.isfinite(t) or abs(t-expected_time)>1e-5:raise ValueError('Exposure and native physics time differ')
 if rgba.shape!=(768,1024,4) or rgba.dtype!=np.uint8 or depth.shape!=(768,1024):raise ValueError('Native format mismatch')
 if np.isnan(depth).any():raise ValueError('NaN renderer labels')
 if not isinstance(p,dict):raise ValueError('CameraParams mapping missing')
 if not np.array_equal(np.asarray(p['renderProductResolution']),[1024,768]):raise ValueError('Render product resolution differs')
 focal=float(p['cameraFocalLength'])/float(np.asarray(p['cameraAperture'])[0])*1024
 if abs(focal-expected_focal_px)>2e-4:raise ValueError('Exposure focal calibration differs')
 if abs(float(p['metersPerSceneUnit'])-1)>1e-6:raise ValueError('Scene scale differs from metric contract')
 view=np.asarray(p['cameraViewTransform'],float).reshape(4,4)
 if not np.isfinite(view).all() or abs(np.linalg.det(view[:3,:3])-1)>2e-5:raise ValueError('Renderer view transform invalid')
 # Installed OgnPointCloudGenerator explicitly inverts world-to-view row-vector matrix.
 T=np.linalg.inv(view).T
 after=camera.get_current_frame()
 if frame!=int(after['rendering_frame']) or t!=float(after['rendering_time']):raise ValueError('Frame changed during copy')
 return dict(rgb=rgba[:,:,:3].copy(),depth_label=depth.copy(),render_frame=frame,render_time=t,camera_usd_world_label=T,params=p)

def paired_snapshot(cameras,previous,expected_time,focal):
 rows=[snapshot(c,p,expected_time,focal) for c,p in zip(cameras,previous)]
 if len(rows)!=2 or abs(rows[0]['render_time']-rows[1]['render_time'])>1e-5:raise ValueError('Unpaired camera exposure times')
 return rows
