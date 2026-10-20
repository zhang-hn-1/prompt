"""Catch global-material writes, lost face colours, unsafe anchors and impossible budget."""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/an/miniconda3/envs/ball_wm/lib/python3.10/site-packages/isaacsim/extscache/omni.usd.libs-1.0.1+d02c707b.lx64.r.cp310')
from pxr import Usd,UsdGeom,UsdLux,UsdShade,Gf
import board_material

def test_matte_board_binding_preserves_colours_geometry_and_other_surfaces():
    stage=Usd.Stage.CreateInMemory();board=UsdGeom.Mesh.Define(stage,'/World/CalibrationOnlyMovingCodedBoard')
    board.CreatePointsAttr([Gf.Vec3f(2,0,0),Gf.Vec3f(2,1,0),Gf.Vec3f(2,1,1),Gf.Vec3f(2,0,1)])
    board.CreateFaceVertexCountsAttr([4]);board.CreateFaceVertexIndicesAttr([0,1,2,3]);board.CreateDisplayColorAttr([Gf.Vec3f(.01,.01,.01)])
    UsdGeom.Primvar(board.GetDisplayColorAttr()).SetInterpolation('uniform')
    floor=UsdGeom.Mesh.Define(stage,'/World/floor');lamp=UsdLux.SphereLight.Define(stage,'/World/lamp');lamp.CreateIntensityAttr(123.)
    original=[board.GetPointsAttr().Get(),board.GetFaceVertexIndicesAttr().Get(),board.GetDisplayColorAttr().Get()]
    result=board_material.apply_matte_board(stage,board)
    assert original==[board.GetPointsAttr().Get(),board.GetFaceVertexIndicesAttr().Get(),board.GetDisplayColorAttr().Get()]
    assert UsdGeom.Primvar(board.GetDisplayColorAttr()).GetInterpolation()=='uniform'
    assert not floor.GetPrim().GetRelationship('material:binding').GetTargets() and lamp.GetIntensityAttr().Get()==123.
    bound,_=UsdShade.MaterialBindingAPI(board).ComputeBoundMaterial();assert str(bound.GetPath())==result['material_path']
    shader=UsdShade.Shader(stage.GetPrimAtPath(result['shader_path']))
    assert shader.GetIdAttr().Get()=='UsdPreviewSurface'
    assert shader.GetInput('roughness').Get()==1. and shader.GetInput('useSpecularWorkflow').Get()==1
    assert shader.GetInput('specularColor').Get()==Gf.Vec3f(0,0,0) and shader.GetInput('metallic').Get()==0
    source,name,_=shader.GetInput('diffuseColor').GetConnectedSource()
    assert name=='result' and UsdShade.Shader(source).GetInput('varname').Get()=='displayColor'
    assert not stage.GetPrimAtPath('/World').GetRelationship('material:binding').GetTargets()

def test_material_rejects_noncalibration_target_before_stage_mutation():
    stage=Usd.Stage.CreateInMemory();mesh=UsdGeom.Mesh.Define(stage,'/World/Obstacle');before=stage.GetRootLayer().ExportToString()
    import pytest
    with pytest.raises(ValueError):board_material.apply_matte_board(stage,mesh)
    assert before==stage.GetRootLayer().ExportToString()

def test_anchor_selection_retains_safe_unique_rgb_points_without_labels():
    ids=np.array([2,0]);corners=np.array([[[20,20],[30,20],[30,30],[20,30]],[[1,1],[20,20],[50,50],[60,60]]],float)
    mask=np.zeros((80,80),bool);mask[49,50]=True
    anchors,labels=board_material.select_rgb_anchors(ids,corners,mask,24)
    np.testing.assert_equal(anchors,[[20,20],[60,60],[20,30],[30,30],[30,20]])
    assert labels==[[0,1],[0,3],[2,1],[2,2],[2,3]]

def test_conditional_budget_cannot_have_unreachable_minimum():
    import pytest
    with pytest.raises(ValueError):board_material.validate_budget(6,12,100)
    assert board_material.validate_budget(6,24,100)==144
