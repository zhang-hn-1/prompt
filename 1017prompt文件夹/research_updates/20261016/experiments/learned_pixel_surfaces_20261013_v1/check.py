from common import *
import numpy as np,torch
from model import PixelSurface
from causal_input import features
R=GROUPS['E0'];f=np.load(R/'source_inputs/frame_000.npz');n=np.load(R/'known_poses/frame_000.npz')
x=features(f['left_rgb'],f['right_rgb'],n['left_camera_usd_world'],n['right_camera_usd_world']);assert x.shape==(10,240,320) and np.isfinite(x).all()
# Causal features ignore native translation; only calibrated orientation used.
a=n['left_camera_usd_world'].copy();b=n['right_camera_usd_world'].copy();a[:3,3]+=100;b[:3,3]-=100
assert np.array_equal(x,features(f['left_rgb'],f['right_rgb'],a,b))
torch.set_num_threads(1);m=PixelSurface();logit,z=m(torch.from_numpy(x)[None]);assert logit.shape==(1,2,3,240,320) and z.shape==(1,2,240,320)
assert ((z>=.5)&(z<=3)).all();loss=logit.square().mean()+z.mean();loss.backward();assert all(p.grad is not None and torch.isfinite(p.grad).all() for p in m.parameters())
save('interface_check.json',dict(pass_check=True,input_shape=list(x.shape),semantic_shape=list(logit.shape),range_shape=list(z.shape),finite_gradient=True,translation_independent_features=True,source_sha256={str(p):sha(p) for p in [D/'causal_input.py',D/'model.py',D/'check.py']}))
