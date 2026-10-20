"""Postsealed independent E1 primitive ray diagnostic; never a decision source."""
from pathlib import Path
import datetime, hashlib, json
import numpy as np
from pxr import Usd, UsdGeom
from scene_ray_geometry import cast_scene

H = Path(__file__).resolve().parent
D = H / 'mvs_hero_temporal_development_v2_api_scale_recovery'
O = H / 'qualified_hazard_navigation_run_v1'
SCENE = Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm/data/benchmark_v1/scenes/benchmark_e1/e1_detour.usda')
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert sha(SCENE) == 'd6236dea995f04e3a81fb90dcde4bde6b63841c283cc5b2470aee8cda34c8634'
stage = Usd.Stage.Open(str(SCENE))
assert UsdGeom.GetStageMetersPerUnit(stage) == 1
assert UsdGeom.GetStageUpAxis(stage) == 'Z'
cache = UsdGeom.XformCache()
assert np.array_equal(np.asarray(cache.GetLocalToWorldTransform(stage.GetDefaultPrim())), np.eye(4))
shapes = []
for prim in stage.Traverse():
    assert not any(a.GetTimeSamples() for a in prim.GetAttributes())
    if not prim.IsA(UsdGeom.Gprim):
        continue
    if UsdGeom.Imageable(prim).ComputeVisibility() == 'invisible':
        continue
    assert prim.IsA(UsdGeom.Cylinder), (prim.GetPath(), prim.GetTypeName())
    T = np.asarray(cache.GetLocalToWorldTransform(prim)).T
    assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3))
    cylinder = UsdGeom.Cylinder(prim)
    assert cylinder.GetAxisAttr().Get() == 'Z'
    shapes.append(dict(type='Cylinder', path=str(prim.GetPath()), centre=T[:3, 3].tolist(),
                       rotation=T[:3, :3].tolist(), radius=float(cylinder.GetRadiusAttr().Get()),
                       height=float(cylinder.GetHeightAttr().Get())))
assert shapes
arrays, rows, inputs = {}, [], {str(SCENE): sha(SCENE)}
for r in json.loads((D / 'records.json').read_text()):
    p = D / r['file']
    truth_path = O / 'offline_truth' / f"frame_{r['frame']:03d}.npz"
    f, gt = np.load(p), np.load(truth_path)
    x, y = f['current_uv'].astype(int).T
    T = gt[r['eye'] + '_camera_usd_world']
    rays = np.c_[(x - 319.5) / 584.8888888888889,
                 (y - 239.5) / 584.8888888888889, np.ones(len(x))]
    world_rays = rays @ (T[:3, :3] @ np.diag([1, -1, -1])).T
    analytic, identity = cast_scene(T[:3, 3], world_rays, shapes, 3., ground_z=0.)
    label = gt[r['eye'] + '_depth_m'][y, x]
    supported = np.isfinite(label)
    hit = np.isfinite(analytic)
    both = supported & hit
    error = np.full(len(label), np.nan)
    error[both] = np.abs(label[both] - analytic[both]) * np.linalg.norm(rays[both], axis=1)
    unsupported_with_hit = ~supported & hit
    ids, counts = np.unique(identity[unsupported_with_hit], return_counts=True)
    rows.append(dict(frame=r['frame'], eye=r['eye'], queries=len(label),
                     current_renderer_insufficient=int((~supported).sum()),
                     insufficient_no_near_scene_hit=int((~supported & ~hit).sum()),
                     insufficient_with_near_scene_hit=int(unsupported_with_hit.sum()),
                     insufficient_hit_primitive_counts={str(int(i)): int(n) for i, n in zip(ids, counts)},
                     finite_renderer_no_near_scene_hit=int((supported & ~hit).sum()),
                     finite_both_world_gap_le01=int((both & (error <= .01)).sum()),
                     finite_both_world_gap_gt01=int((both & (error > .01)).sum()),
                     finite_both_world_gap_median_m=float(np.median(error[both])) if both.any() else None,
                     finite_both_world_gap_p90_m=float(np.quantile(error[both], .9)) if both.any() else None))
    prefix = f"{r['frame']}_{r['eye']}"
    arrays[prefix + '_uv'] = f['current_uv']
    arrays[prefix + '_scene_axial_m'] = analytic
    arrays[prefix + '_scene_primitive_id'] = identity
    arrays[prefix + '_renderer_axial_m'] = label
    inputs[str(p)] = sha(p)
    inputs[str(truth_path)] = sha(truth_path)
target = D / 'missing_support_scene_ray_v1.json'
raw = D / 'missing_support_scene_ray_v1.npz'
assert not target.exists() and not raw.exists()
np.savez_compressed(raw, **arrays)
target.write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                                  rows=rows, shapes=shapes, max_axial_depth_m=3.,
                                  ground_z_proxy_m=0., robot_geometry_not_cast=True,
                                  analytic_primitives_not_renderer_mesh_equivalence=True,
                                  postseal_diagnostic_only=True, old_model_REJECT_preserved=True,
                                  input_sha256=inputs, raw_sha256=sha(raw),
                                  source_sha256={str(Path(__file__)): sha(__file__),
                                                 str(H/'scene_ray_geometry.py'): sha(H/'scene_ray_geometry.py')}), indent=2) + '\n')
print(json.dumps(rows, indent=2))
