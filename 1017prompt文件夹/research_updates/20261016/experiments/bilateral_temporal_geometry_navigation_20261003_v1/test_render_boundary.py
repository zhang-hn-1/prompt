"""Use actual bundled USD; a typeless parent cannot hide its children."""
from pxr import Usd, UsdGeom
import render_boundary


def test_typeless_parent_invisible_does_not_hide_marker():
    stage = Usd.Stage.CreateInMemory()
    marker = UsdGeom.Sphere.Define(stage, '/World/BenchmarkMarkers/Goal')
    root = stage.GetPrimAtPath('/World/BenchmarkMarkers')
    assert not UsdGeom.Imageable(root)
    UsdGeom.Imageable(root).MakeInvisible()
    assert root.GetAttribute('visibility').Get() == 'invisible'
    assert marker.ComputeVisibility() == 'inherited'


def test_hide_actual_marker_geometry_and_check_after_update():
    stage = Usd.Stage.CreateInMemory()
    marker = UsdGeom.Sphere.Define(stage, '/World/BenchmarkMarkers/Goal')
    curve = UsdGeom.BasisCurves.Define(stage, '/World/BenchmarkMarkers/OraclePath')
    scene = UsdGeom.Sphere.Define(stage, '/World/SceneObstacle')
    snapshot = render_boundary.hide_diagnostic_markers(stage, UsdGeom)
    assert set(snapshot) == {str(marker.GetPath()), str(curve.GetPath())}
    assert all(row['computed_visibility'] == 'invisible' for row in snapshot.values())
    curve.CreateCurveVertexCountsAttr([1])
    assert render_boundary.marker_visibility(stage, UsdGeom) == snapshot
    assert scene.ComputeVisibility() == 'inherited'
