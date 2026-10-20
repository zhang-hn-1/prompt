"""Offline CAD projection; runtime loads a frozen scene-independent mask."""
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from pxr import Usd,UsdGeom
from self_mask import mask_triangles
from scripts.camera_rig import camera_translation_base_link, camera_rig_metadata

HERE=Path(__file__).resolve().parent
ASSET=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/project/rl_env_0.4/rl_env/rotunbot_wheel_sim_model_0.4/Rotunbot_wheel.usd')


def main():
    stage=Usd.Stage.Open(str(ASSET));cache=UsdGeom.XformCache()
    base=stage.GetPrimAtPath('/Rotunbot_wheel/base_link')
    polygons=[]
    for prim in Usd.PrimRange(base):
        if not prim.IsA(UsdGeom.Mesh):continue
        mesh=UsdGeom.Mesh(prim)
        if mesh.ComputeVisibility()=='invisible':continue
        points=np.asarray(mesh.GetPointsAttr().Get())
        matrix=np.asarray(cache.GetLocalToWorldTransform(prim)*cache.GetLocalToWorldTransform(base).GetInverse())
        points=(np.c_[points,np.ones(len(points))]@matrix)[:,:3]
        counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get());indices=np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
        assert np.all(counts==3),'CAD triangulation must be explicit'
        polygons.append(points[indices.reshape(-1,3)])
    triangles=np.vstack(polygons)
    rotation=np.array([[0,0,1],[0,-1,0],[1,0,0]])
    masks={};metadata={}
    for eye,y in [('left',.3),('right',-.3)]:
        optical=(triangles-np.array(camera_translation_base_link(y)))@rotation
        mask=mask_triangles(optical,640,480,16.45/18*640,319.5,239.5)
        dilated=cv2.dilate(mask.astype(np.uint8),np.ones((7,7),np.uint8)).astype(bool)
        masks[eye]=dilated;metadata[eye]=dict(raw_mask_pixels=int(mask.sum()),mask_pixels=int(dilated.sum()),
                        image_fraction=float(dilated.mean()))
    np.savez_compressed(HERE/'base_link_masks.npz',**masks)
    layers={l.realPath:hashlib.sha256(Path(l.realPath).read_bytes()).hexdigest() for l in stage.GetUsedLayers() if l.realPath and Path(l.realPath).is_file()}
    (HERE/'mask_identity.json').write_text(json.dumps(dict(asset=str(ASSET),source_layers_sha256=layers,
        camera_rig=camera_rig_metadata(),covered_body='base_link only, rigidly fixed to both cameras',triangles=len(triangles),
        dilation_pixels=3,scene_inputs_used=False,eyes=metadata,
        mask_sha256=hashlib.sha256((HERE/'base_link_masks.npz').read_bytes()).hexdigest()),indent=2)+'\n')
    print(metadata)


if __name__=='__main__':main()
