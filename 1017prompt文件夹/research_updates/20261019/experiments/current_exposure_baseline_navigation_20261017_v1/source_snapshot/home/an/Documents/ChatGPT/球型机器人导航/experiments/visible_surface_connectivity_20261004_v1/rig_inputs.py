"""New-rig measurement guard. No policy/model has access to future truth."""
import numpy as np
RIG_ID='ball_stereo_front020_up005_v2'

def validate_mask_identity(identity):
    if identity.get('camera_rig',{}).get('rig_id')!=RIG_ID:
        raise ValueError('mask rig identity mismatch')
    return True


def validate_capture(row):
    rotation=np.array([[0,0,1],[0,-1,0],[1,0,0]])
    for eye,y in [('left',.3),('right',-.3)]:
        matrix=np.asarray(row[eye+'_optical_to_base'])
        if matrix.shape!=(4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[:3,3],[.2,y,.05],atol=2e-5,rtol=0) or not np.allclose(matrix[:3,:3],rotation,atol=2e-5,rtol=0):
            raise ValueError('measured camera rig mismatch')
        rgb=np.asarray(row[eye+'_rgb'])
        if rgb.shape!=(480,640,3) or rgb.dtype!=np.uint8:raise ValueError('image shape/dtype mismatch')
        times=np.array([row['raw_time_s'],row[eye+'_render_time_s']],dtype=float)
        if not np.isfinite(times).all() or abs(times[0]-times[1])>1e-5:raise ValueError('image/physics time mismatch')
    return True
