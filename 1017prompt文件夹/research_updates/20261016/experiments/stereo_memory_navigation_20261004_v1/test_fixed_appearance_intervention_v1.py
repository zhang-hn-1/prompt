from pathlib import Path
import numpy as np
import pytest
import subprocess,sys,os
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade
from fixed_appearance_intervention_v1 import bind_fixed_appearance,make_fixed_texture
H=Path(__file__).resolve().parent

def test_material_module_bootstraps_before_simulationapp():
 env=dict(os.environ,PYTHONPATH='/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
 result=subprocess.run([sys.executable,'-c','import fixed_appearance_intervention_v1'],cwd=H,env=env,capture_output=True,text=True)
 assert result.returncode==0,result.stderr

def test_geometry_physics_and_material_scope(tmp_path):
 st=Usd.Stage.Open(str(H/'controlled_obstacle_fixture_v1/single_cylinder.usda'))
 p=st.GetPrimAtPath('/ControlledObstacle/obstacle');before={a.GetName():a.Get() for a in p.GetAttributes()}
 texture=tmp_path/'fixed.png';make_fixed_texture(texture)
 evidence=bind_fixed_appearance(st,texture)
 assert {a.GetName():a.Get() for a in p.GetAttributes()}==before
 assert p.GetTypeName()=='Cylinder' and p.HasAPI(UsdPhysics.CollisionAPI)
 assert UsdGeom.Cylinder(p).GetRadiusAttr().Get()==.35
 assert evidence['original_collision_attributes_unchanged']
 mat,_=UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial();assert str(mat.GetPath())=='/ResearchFixedAppearance'
 shader=UsdShade.Shader(st.GetPrimAtPath('/ResearchFixedAppearance/shader'))
 assert shader.GetInput('project_uvw').Get() is True
 assert shader.GetInput('enable_emission').Get() is False
 assert shader.GetInput('metallic_constant').Get()==0
 assert abs(shader.GetInput('albedo_brightness').Get()-.27)<1e-7
 assert shader.GetInput('diffuse_texture').Get().path==str(texture)

def test_texture_reproducible_and_explicit_domain(tmp_path):
 a=make_fixed_texture(tmp_path/'a.png');b=make_fixed_texture(tmp_path/'b.png')
 np.testing.assert_array_equal(a,b);assert a.shape==(512,512,3)
 assert a.min()>=173 and a.max()<=255 and np.std(a)>2
 assert np.array_equal(a[:,:,0],a[:,:,1]) and np.array_equal(a[:,:,0],a[:,:,2])

def test_requires_unique_unchanged_cylinder(tmp_path):
 tex=tmp_path/'a.png';make_fixed_texture(tex)
 st=Usd.Stage.CreateInMemory()
 with pytest.raises(ValueError):bind_fixed_appearance(st,tex)
 for n in ['a','b']:
  p=UsdGeom.Cylinder.Define(st,'/'+n+'/obstacle');p.CreateRadiusAttr(.35);p.CreateHeightAttr(1.)
 with pytest.raises(ValueError):bind_fixed_appearance(st,tex)
