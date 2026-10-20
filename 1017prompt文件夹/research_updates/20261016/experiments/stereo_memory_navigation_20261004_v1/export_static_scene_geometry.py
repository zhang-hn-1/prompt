"""Read authored USD geometry without initializing or stepping a simulator."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
H = Path(__file__).resolve().parent
USD_EXTENSION = Path('/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')


def main():
    from pxr import Usd, UsdGeom, UsdPhysics
    protocol = json.loads((H/'scene_ray_protocol.json').read_text())
    scene = Path('/home/an/project/NavDP/assets/scenes/cluttered_hard/hard_0/cluttered-0.usd')
    assert hashlib.sha256(scene.read_bytes()).hexdigest() == protocol['input_sha256'][str(scene)]
    stage = Usd.Stage.Open(str(scene)); cache = UsdGeom.XformCache()
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1. and UsdGeom.GetStageUpAxis(stage) == 'Z'
    assert np.array_equal(np.asarray(cache.GetLocalToWorldTransform(stage.GetDefaultPrim())), np.eye(4))
    assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse())
    assert not any(a.GetTimeSamples() for p in stage.Traverse() for a in p.GetAttributes())
    shapes = []; mesh_arrays = {}; visible_geometry = []
    ground = UsdGeom.Mesh(stage.GetPrimAtPath('/World/defaultGroundPlane'))
    ground_points = np.asarray(ground.GetPointsAttr().Get())
    assert np.all(ground_points[:, 2] == 0.)
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(prim).ComputeVisibility() != 'invisible':
            visible_geometry.append(str(prim.GetPath()))
        if prim.GetTypeName() not in ['Sphere', 'Cylinder'] or UsdGeom.Imageable(prim).ComputeVisibility() == 'invisible': continue
        matrix = np.asarray(cache.GetLocalToWorldTransform(prim)).T
        rotation = matrix[:3, :3]
        assert np.allclose(rotation.T@rotation, np.eye(3), atol=1e-8) and np.linalg.det(rotation) > .999999
        geometry = UsdGeom.Sphere(prim) if prim.GetTypeName() == 'Sphere' else UsdGeom.Cylinder(prim)
        row = dict(path=str(prim.GetPath()), type=prim.GetTypeName(), centre=matrix[:3, 3].tolist(),
                   rotation=rotation.tolist(), radius=float(geometry.GetRadiusAttr().Get()), visible=True, mesh_checks=[])
        if row['type'] == 'Cylinder':
            assert geometry.GetAxisAttr().Get() == 'Z'; row['height'] = float(geometry.GetHeightAttr().Get())
        for child in prim.GetChildren():
            if child.GetTypeName() != 'Mesh': continue
            mesh = UsdGeom.Mesh(child); points = np.asarray(mesh.GetPointsAttr().Get())
            transform = np.asarray(cache.GetLocalToWorldTransform(child)).T
            assert np.allclose(transform, matrix, atol=1e-8)
            index = len(shapes); prefix = f'shape_{index:03d}'
            mesh_arrays[prefix+'_points_local'] = points
            mesh_arrays[prefix+'_face_counts'] = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
            mesh_arrays[prefix+'_face_indices'] = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
            radial = np.linalg.norm(points, axis=-1) if row['type'] == 'Sphere' else np.linalg.norm(points[:, :2], axis=-1)
            row['mesh_checks'].append(dict(path=str(child.GetPath()), point_n=len(points),
                radial_min_m=float(radial.min()), radial_max_m=float(radial.max()),
                max_radius_deviation_m=float(np.max(np.abs(radial-row['radius']))),
                subdivision_scheme=str(mesh.GetSubdivisionSchemeAttr().Get()),
                z_extent_m=[float(points[:, 2].min()), float(points[:, 2].max())]))
            # Cylinder cap-center vertices need not be on the rim.
            if row['type'] == 'Sphere': assert np.max(np.abs(radial-row['radius'])) < 1e-5
            else: assert abs(radial.max()-row['radius']) < 1e-5 and abs(np.ptp(points[:, 2])-row['height']) < 1e-5
        assert row['mesh_checks']
        shapes.append(row)
    expected = {s['path'] for s in shapes} | {m['path'] for s in shapes for m in s['mesh_checks']} | {'/World/defaultGroundPlane'}
    assert set(visible_geometry) == expected, set(visible_geometry)-expected
    np.savez_compressed(H/'static_scene_child_meshes.npz', **mesh_arrays)
    result = dict(scene=str(scene), scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        role='independent authored static geometry labels; analytic visible prims + checked authored child meshes; not full subdivided renderer equivalence',
        root_matrix_identity=True, units_m=1., up_axis='Z', rigid_bodies=0, time_sampled_attributes=0,
        ground_z_m=0., ground_xy_min_m=ground_points[:, :2].min(axis=0).tolist(),
        ground_xy_max_m=ground_points[:, :2].max(axis=0).tolist(), shapes=shapes,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__), USD_EXTENSION/'pxr/Usd/_usd.so']})
    (H/'static_scene_geometry.json').write_text(json.dumps(result, indent=2)+'\n')
    print('EXPORTED', len(shapes), 'visible analytic shapes; child mesh dimensions verified; no physics')


if __name__ == '__main__':
    sys.path.insert(0, str(USD_EXTENSION))
    main()
