"""A single calibration-board material intervention, no light/camera/control writes."""
import numpy as np
BOARD_PATH='/World/CalibrationOnlyMovingCodedBoard'

def apply_matte_board(stage,mesh):
    from pxr import UsdShade,Sdf,Gf
    if str(mesh.GetPath())!=BOARD_PATH:raise ValueError('only the standalone calibration board may be bound')
    root=BOARD_PATH+'/MatteMaterial';material=UsdShade.Material.Define(stage,root)
    shader=UsdShade.Shader.Define(stage,root+'/Surface');shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.)
    shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
    shader.CreateInput('useSpecularWorkflow',Sdf.ValueTypeNames.Int).Set(1)
    shader.CreateInput('specularColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0,0,0))
    shader.CreateInput('clearcoat',Sdf.ValueTypeNames.Float).Set(0.)
    shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(1.)
    reader=UsdShade.Shader.Define(stage,root+'/FaceColor');reader.CreateIdAttr('UsdPrimvarReader_float3')
    reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('displayColor')
    reader.CreateInput('fallback',Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(.01,.01,.01))
    reader.CreateOutput('result',Sdf.ValueTypeNames.Float3)
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(reader.ConnectableAPI(),'result')
    shader.CreateOutput('surface',Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    bound,_=UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial();assert bound.GetPath()==material.GetPath()
    return dict(material_path=root,shader_path=str(shader.GetPath()),shader_id='UsdPreviewSurface',
                roughness=shader.GetInput('roughness').Get(),metallic=shader.GetInput('metallic').Get(),
                useSpecularWorkflow=shader.GetInput('useSpecularWorkflow').Get(),
                specularColor=list(shader.GetInput('specularColor').Get()),clearcoat=shader.GetInput('clearcoat').Get(),
                diffuseColor_source='UsdPrimvarReader_float3/displayColor uniform existing face values',
                opacity=shader.GetInput('opacity').Get(),scope='board mesh only; no emission/light/camera/physics writes')

def validate_budget(frames,maximum,min_samples):
    budget=frames*maximum
    if frames<1 or maximum<1 or min_samples<1 or budget<min_samples:raise ValueError('unreachable conditional sample gate')
    return budget

def select_rgb_anchors(ids,corners,mask,maximum):
    mask=np.asarray(mask,bool);height,width=mask.shape;candidates=[];labels=[];seen=set()
    for index in np.argsort(ids):
        for j,point in enumerate(corners[index]):
            if not np.isfinite(point).all():continue
            x,y=np.rint(point).astype(int)
            if x<2 or x>=width-2 or y<2 or y>=height-2 or mask[y-2:y+3,x-2:x+3].any() or (y,x) in seen:continue
            seen.add((y,x));candidates.append((y,x));labels.append([int(ids[index]),j])
    ranks=np.linspace(0,len(candidates)-1,min(maximum,len(candidates)),dtype=int) if candidates else np.array([],int)
    return np.array(candidates,int).reshape(-1,2)[ranks],[labels[i] for i in ranks]
