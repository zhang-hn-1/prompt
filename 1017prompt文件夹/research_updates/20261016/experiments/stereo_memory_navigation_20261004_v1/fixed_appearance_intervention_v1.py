"""One explicitly separate appearance domain; no physics or decision input."""
from pathlib import Path
import hashlib,numpy as np,cv2
def make_fixed_texture(path):
 path=Path(path)
 if path.exists():raise FileExistsError(path)
 rng=np.random.default_rng(20261008)
 gray=np.clip(np.rint(214+rng.normal(0,24,(128,128))),173,255).astype(np.uint8)
 gray=cv2.resize(gray,(512,512),interpolation=cv2.INTER_NEAREST);rgb=np.repeat(gray[:,:,None],3,axis=2)
 assert cv2.imwrite(str(path),rgb)
 return rgb
def bind_fixed_appearance(stage,texture):
 # Called from the existing marker hook after SimulationApp loads USD.
 # The GPU launch environment cannot import pxr before that initialization.
 from pxr import UsdShade,Sdf,Gf,UsdGeom,UsdPhysics
 texture=Path(texture).resolve();assert texture.is_file()
 targets=[p for p in stage.Traverse() if p.GetName()=='obstacle' and p.GetTypeName()=='Cylinder']
 if len(targets)!=1:raise ValueError('exactly one named analytic obstacle cylinder required')
 p=targets[0];c=UsdGeom.Cylinder(p)
 if c.GetRadiusAttr().Get()!=.35 or c.GetHeightAttr().Get()!=1. or not p.HasAPI(UsdPhysics.CollisionAPI):raise ValueError('frozen collision cylinder required')
 before={a.GetName():str(a.Get()) for a in p.GetAttributes()};api_before=list(p.GetAppliedSchemas())
 path='/ResearchFixedAppearance'
 if stage.GetPrimAtPath(path):raise ValueError('no existing research material overwritten')
 material=UsdShade.Material.Define(stage,path);shader=UsdShade.Shader.Define(stage,path+'/shader');shader.CreateIdAttr('OmniPBR');shader.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset);shader.SetSourceAsset(Sdf.AssetPath('OmniPBR.mdl'),'mdl');shader.SetSourceAssetSubIdentifier('OmniPBR','mdl')
 shader.CreateInput('diffuse_color_constant',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.18,.18,.18))
 # SDK MDL defines brightness as an albedo multiplier; constant base color
 # is not assumed to multiply a supplied texture. Fixed before any render.
 shader.CreateInput('albedo_brightness',Sdf.ValueTypeNames.Float).Set(.27)
 shader.CreateInput('reflection_roughness_constant',Sdf.ValueTypeNames.Float).Set(.5)
 shader.CreateInput('metallic_constant',Sdf.ValueTypeNames.Float).Set(0.)
 shader.CreateInput('enable_emission',Sdf.ValueTypeNames.Bool).Set(False)
 shader.CreateInput('diffuse_texture',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
 shader.CreateInput('project_uvw',Sdf.ValueTypeNames.Bool).Set(True)
 shader.CreateInput('texture_scale',Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(2.,2.))
 shader.CreateInput('texture_translate',Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.,0.))
 material.CreateSurfaceOutput('mdl').ConnectToSource(shader.CreateOutput('out',Sdf.ValueTypeNames.Token))
 UsdShade.MaterialBindingAPI.Apply(p).Bind(material)
 after={a.GetName():str(a.Get()) for a in p.GetAttributes()};assert after==before and p.HasAPI(UsdPhysics.CollisionAPI)
 return dict(obstacle_path=str(p.GetPath()),material_path=path,texture_sha256=hashlib.sha256(texture.read_bytes()).hexdigest(),original_collision_attributes_unchanged=True,geometry_attributes_before=before,geometry_attributes_after=after,api_before=api_before,api_after=list(p.GetAppliedSchemas()),appearance_domain_separate_from_old_uniform=True,not_isolated_texture_only_cause=True,no_control_or_truth_inputs=True,no_emission_or_displacement=True)
