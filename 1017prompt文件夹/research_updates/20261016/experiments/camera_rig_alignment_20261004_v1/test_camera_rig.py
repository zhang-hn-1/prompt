"""Exercise the camera constructors without importing or starting Isaac Sim."""
import argparse
import ast
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace
import pytest

DEFAULT_ROOT = Path(__file__).resolve().parents[1] if Path(__file__).parent.name == 'tests' else Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
ROOT = Path(os.environ.get('CAMERA_PROJECT_ROOT', str(DEFAULT_ROOT)))
CASES = [('scripts/collect_data_dubins.py','add_camera_prim'),
         ('training/evaluate_navigation.py','add_camera_prim'),
         ('benchmark/isaac_runtime.py','add_stereo_camera'),
         ('scripts/collect_data.py','_make_cam_usd'),
         ('scripts/sim_env.py','_make_camera')]

class Prim:
    def __init__(self): self.values = {}
    def __getattr__(self, name):
        if name == 'ClearXformOpOrder': return lambda: [self.values.pop(k,None) for k in ('AddTranslateOp','AddRotateYZXOp')]
        return lambda: SimpleNamespace(Set=lambda v: self.values.__setitem__(name, tuple(v) if isinstance(v, (tuple,list)) else v))


def namespace(tree):
    cameras = {}
    def define(stage, path):
        cameras[path] = Prim(); return cameras[path]
    env = dict(width=640,height=480,focal_length=16.45,horizontal_aperture=18.0,stage=None, UsdGeom=SimpleNamespace(Camera=SimpleNamespace(Define=define),Xformable=lambda p:p),
               Gf=SimpleNamespace(Vec3d=lambda *v:v,Vec3f=lambda *v:v,Vec2f=lambda *v:v),
               args=SimpleNamespace(img_h=480,img_w=640),
               self=SimpleNamespace(_contract=SimpleNamespace(image_height_px=480,image_width_px=640)),
               LEFT_CAMERA_PATH='left',RIGHT_CAMERA_PATH='right')
    helper = ROOT/'scripts/camera_rig.py'
    if helper.exists():
        spec=importlib.util.spec_from_file_location('rig_for_test',helper)
        rig=importlib.util.module_from_spec(spec); spec.loader.exec_module(rig)
        env.update(vars(rig))
    names={'camera_x','camera_y','camera_z','CAM_X','CAM_Y','CAM_Z','focal_length','horizontal_aperture',
           'vertical_aperture','FOCAL_LEN','H_APT','V_APT','CAM_W','CAM_H','H_APERTURE','V_APERTURE'}
    for node in ast.walk(tree):
        if isinstance(node,ast.Assign):
            targets={n.id for target in node.targets for n in ast.walk(target) if isinstance(n,ast.Name)}
            if targets and targets.issubset(names):
                exec(compile(ast.Module(body=[node],type_ignores=[]),'<assign>','exec'),env)
    return env,cameras

@pytest.mark.parametrize('relative,function',CASES)
def test_live_stereo_transforms(relative,function):
    tree=ast.parse((ROOT/relative).read_text()); env,cameras=namespace(tree)
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==function)
    exec(compile(ast.Module(body=[node],type_ignores=[]),relative,'exec'),env)
    if function=='_make_camera':
        parser=argparse.ArgumentParser()
        for n in tree.body:
            if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call) and n.value.args and isinstance(n.value.args[0],ast.Constant) and n.value.args[0].value in ('--cam_x','--cam_y','--cam_z'):
                exec(compile(ast.Module(body=[n],type_ignores=[]),relative,'exec'),dict(env,parser=parser))
        args=parser.parse_args([])
        left=(args.cam_x,args.cam_y,args.cam_z); right=(args.cam_x,-args.cam_y,args.cam_z)
        env[function]('left',*left);env[function]('right',*right)
        override=parser.parse_args(['--cam_x','0.21','--cam_z','0.06'])
        assert (override.cam_x,override.cam_z)==(.21,.06)
    else:
        y=env.get('camera_y',env.get('CAM_Y',env.get('CAMERA_HALF_BASELINE_M',.3)))
        env[function]('left',y);env[function]('right',-y)
    assert cameras['left'].values['AddTranslateOp']==pytest.approx((.2,.3,.05))
    assert cameras['right'].values['AddTranslateOp']==pytest.approx((.2,-.3,.05))
    for camera in cameras.values():
        assert camera.values['AddRotateYZXOp']==(0.,-90.,0.)
        assert camera.values['GetFocalLengthAttr']==16.45
        assert camera.values['GetHorizontalApertureAttr']==18.
        assert camera.values['GetVerticalApertureAttr']==13.5
