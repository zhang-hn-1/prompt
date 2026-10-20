from common import *
import numpy as np,torch,time
from joint import JointStereo,alignment
from test_joint import main

checks=main();rng=np.random.default_rng(5577)
c=rng.integers(0,200,(3,12,12),dtype=np.int32);g=rng.integers(0,55,(3,12),dtype=np.int32)
a=alignment(torch.from_numpy(c),torch.from_numpy(g),torch.from_numpy(g))
b=alignment(torch.from_numpy(c).cuda(),torch.from_numpy(g).cuda(),torch.from_numpy(g).cuda())
assert all(torch.equal(x,y.cpu()) for x,y in zip(a,b))
l=rng.integers(0,256,(15,20),dtype=np.uint8);r=rng.integers(0,256,l.shape,dtype=np.uint8);m=np.zeros(l.shape,bool)
cpu=JointStereo('cpu').match_rectified(l,r,m,m);gpu=JointStereo('cuda').match_rectified(l,r,m,m)
assert all(np.array_equal(cpu[k],gpu[k]) for k in cpu)
R,cp=GROUPS['E0'];p=R/'source_inputs/frame_000.npz';cfg=json.loads(cp.read_text());masks=dict(np.load(cfg['static_masks']));input=dict(np.load(p))
t=time.monotonic();pred=JointStereo('cuda').predict(input['left_rgb'],input['right_rgb'],masks['left'],masks['right']);torch.cuda.synchronize();elapsed=time.monotonic()-t
np.savez_compressed(D/'preflight_frame000.npz',**pred)
save('preflight_checks.json',dict(**checks,alignment_cpu_gpu_exact=True,pair_cpu_gpu_exact=True,full_native_pair_wall_s=elapsed,predicted_native_pixels=int(pred['confidence'].sum()),input_sha256=sha(p),truth_read=False,physical=0,fit=0))
print(json.dumps(json.loads((D/'preflight_checks.json').read_text())),flush=True)
