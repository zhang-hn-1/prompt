"""Remove rendered diagnostic hints; no physical or policy changes."""
def marker_visibility(stage, usd_geom):
    from pxr import Usd
    root = stage.GetPrimAtPath('/World/BenchmarkMarkers')
    assert root, 'benchmark marker root missing'
    result = {}
    for prim in Usd.PrimRange(root):
        imageable = usd_geom.Imageable(prim)
        if imageable:
            result[str(prim.GetPath())] = dict(
                prim_type=prim.GetTypeName(),
                authored_visibility=str(imageable.GetVisibilityAttr().Get()),
                computed_visibility=str(imageable.ComputeVisibility()))
    assert result, 'no diagnostic geometry to verify'
    return result


def hide_diagnostic_markers(stage, usd_geom):
    for path in marker_visibility(stage, usd_geom):
        usd_geom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()
    result = marker_visibility(stage, usd_geom)
    assert all(r['computed_visibility'] == 'invisible' for r in result.values())
    return result
