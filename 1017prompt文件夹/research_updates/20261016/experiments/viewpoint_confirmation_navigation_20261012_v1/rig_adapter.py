"""Explicit stereo calibration adapter; original source files remain immutable."""
from dataclasses import dataclass
import time
import numpy as np
import cv2
from dynamic_self_mask import RobotSelfMask, joints_from_proprio
from self_mask import mask_triangles
from stereo_geometry import SGBMStereo as OriginalStereo, FOCAL, BASELINE
from native_pose_consumer_v1 import current_native_pose as original_native_pose
from dense_source_points import point_packet
ROTATION=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
@dataclass(frozen=True)
class Rig:
    name:str
    forward:float
    half_baseline:float
    up:float=.05
    @property
    def baseline(self):return 2*self.half_baseline
    def translation(self,eye):
        if eye not in ('left','right'):raise ValueError('eye')
        return np.array([self.forward,self.half_baseline if eye=='left' else -self.half_baseline,self.up])
    def optical(self,eye):
        T=np.eye(4);T[:3,:3]=ROTATION;T[:3,3]=self.translation(eye);return T
    def metadata(self):return dict(rig_id='nearfield_'+self.name+'_20261012_v1',parent_frame='base_link',units='m',left_position_base_link_m=self.translation('left').tolist(),right_position_base_link_m=self.translation('right').tolist(),rotate_yzx_deg=[0,-90,0])
class RigMask(RobotSelfMask):
    def __init__(self,rig):
        self.rig=rig
        super().__init__()
        # Replace the original frozen rig mask with the exact base CAD projection.
        self.frozen=self.project(np.zeros(3),parts=('base_link',),include_frozen=False)
    def project(self,joints,parts=('base_link','link1','link2','link3'),include_frozen=True):
        begin=time.monotonic();transforms=self.body_transforms(joints);tri=[]
        for name in parts:
            T=transforms[name];tri.append(self.triangles[name]@T[:3,:3].T+T[:3,3])
        points=np.vstack(tri);out={}
        for eye in ('left','right'):
            optical=(points-self.rig.translation(eye))@ROTATION
            mask=mask_triangles(optical,640,480,FOCAL,319.5,239.5)
            mask=cv2.dilate(mask.astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool)
            out[eye]=mask|self.frozen[eye] if include_frozen else mask
        self.last_project_wall_s=time.monotonic()-begin
        return out
class RigStereo(OriginalStereo):
    def __init__(self,rig):super().__init__();self.rig=rig
    def predict(self,*args):
        out=super().predict(*args)
        out['depth_m']=out['depth_m']*(self.rig.baseline/BASELINE)
        return out

def native_pose(record,t,rig):
    out=original_native_pose(record,t);R=out['root_rotation_world'];p=out['root_position_world_m']
    for eye in ('left','right'):
        T=np.eye(4);T[:3,:3]=R@ROTATION@np.diag([1,-1,-1]);T[:3,3]=p+R@rig.translation(eye);out[eye+'_camera_usd_world']=T
    return out

def paired(prediction,pose,history,cad,t):
    if float(prediction['time_s'])!=float(t) or not np.isfinite(t):raise ValueError('source clock')
    m=cad.project(joints_from_proprio(history))
    return (point_packet(prediction,pose,cad.frozen['left'],cad.frozen['right'],baseline=cad.rig.baseline),point_packet(prediction,pose,m['left'],m['right'],baseline=cad.rig.baseline),m)

def validate(row,rig):
    for eye in ('left','right'):
        T=np.asarray(row[eye+'_optical_to_base'])
        if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T,rig.optical(eye),atol=2e-5,rtol=0):raise ValueError('rig extrinsics mismatch')
        rgb=np.asarray(row[eye+'_rgb'])
        if rgb.shape!=(480,640,3) or rgb.dtype!=np.uint8:raise ValueError('RGB format')
        ts=np.array([row['raw_time_s'],row[eye+'_render_time_s']],float)
        if not np.isfinite(ts).all() or abs(ts[0]-ts[1])>1e-5:raise ValueError('image clock')
    return True

def install_runtime_geometry(rig):
    import benchmark.isaac_runtime as runtime
    import scripts.camera_rig as metadata
    runtime.CAMERA_HALF_BASELINE_M=rig.half_baseline
    runtime.camera_translation_base_link=lambda y:(rig.forward,float(y),rig.up)
    metadata.CAMERA_RIG_ID=rig.metadata()['rig_id'];metadata.CAMERA_FORWARD_M=rig.forward
    metadata.CAMERA_HALF_BASELINE_M=rig.half_baseline;metadata.CAMERA_UP_M=rig.up
