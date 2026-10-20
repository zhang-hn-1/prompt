"""One frozen official binocular model; both inputs are RGB, never depth truth."""
from pathlib import Path
import time
import numpy as np
from checkpoint_loader import load_model
from stereo_geometry import rectify_pair,restore_native,disparity_depth

class FastStereo:
    def __init__(self):
        import torch
        self.torch=torch
        repo=Path(__file__).parent/'third_party/fast_foundation_stereo'
        self.model=load_model(repo).cuda().eval()
        self.model.args.max_disp=416;self.model.args.valid_iters=8
        from core.utils.utils import InputPadder
        self.Padder=InputPadder
        self.warmup_s=None
    def infer(self,left,right):
        torch=self.torch
        tensors=[torch.from_numpy(np.ascontiguousarray(x)).cuda().float()[None].permute(0,3,1,2) for x in (left,right)]
        padder=self.Padder(tensors[0].shape,divis_by=32,force_square=False)
        a,b=padder.pad(*tensors)
        with torch.inference_mode(),torch.amp.autocast('cuda',dtype=torch.float16):
            d=self.model.forward(a,b,iters=8,test_mode=True,optimize_build_volume='pytorch1')
        return padder.unpad(d.float()).cpu().numpy().reshape(left.shape[:2])
    def predict(self,left,right,left_mask,right_mask):
        l,r=rectify_pair(left,right);lm,rm=rectify_pair(left_mask,right_mask)
        if self.warmup_s is None:
            start=time.perf_counter();self.infer(l,r);self.torch.cuda.synchronize();self.warmup_s=time.perf_counter()-start
            print('FAST_WARMUP_COMPLETE',self.warmup_s,flush=True)
        d=self.infer(l,r)
        # Mirrored swapped pair produces positive disparity for physical right eye.
        dr=-self.infer(np.ascontiguousarray(r[:,::-1]),np.ascontiguousarray(l[:,::-1]))[:,::-1]
        z,valid=disparity_depth(d,dr,lm,rm)
        return dict(depth_m=restore_native(z),confidence=restore_native(valid),disparity_px=d)
