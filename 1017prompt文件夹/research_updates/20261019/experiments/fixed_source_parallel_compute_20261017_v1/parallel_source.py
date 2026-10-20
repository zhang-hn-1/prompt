"""Same four signed SGBM computations; only their execution schedule is parallel."""
from pathlib import Path
import sys,time
from concurrent.futures import ThreadPoolExecutor
S=Path(__file__).resolve().parents[1]/'four_camera_weak_baseline_20261017_v1'
sys.path.insert(0,str(S))
import source as legacy
np=legacy.np;cv2=legacy.cv2
class ParallelFourSGBM(legacy.FourSGBM):
    def __init__(self):
        super().__init__()
        self.matchers=[(self.left,self.right),(cv2.StereoSGBM_create(minDisparity=0,**legacy.CONFIG),cv2.StereoSGBM_create(minDisparity=-448,**legacy.CONFIG))]
        self.pool=ThreadPoolExecutor(max_workers=4)
    def close(self):self.pool.shutdown(wait=True)
    def predict(self,rgb,joints):
        begin=time.monotonic();masks=self.masks(joints);out={};pending=[]
        for pair,(q,p) in enumerate(legacy.PAIRS):
            a,b=[np.ascontiguousarray(np.rot90(rgb[n])) for n in (q,p)]
            lm,rm=[np.ascontiguousarray(np.rot90(masks[n])) for n in (q,p)]
            ap,bp=[cv2.copyMakeBorder(v,0,0,448,448,cv2.BORDER_CONSTANT,value=0) for v in (a,b)]
            ml,mr=self.matchers[pair]
            pending.append((q,p,lm,rm,self.pool.submit(ml.compute,ap,bp),self.pool.submit(mr.compute,bp,ap)))
        for q,p,lm,rm,fl,fr in pending:
            dl=fl.result().astype(np.float32)[:,448:-448]/16.
            dr=fr.result().astype(np.float32)[:,448:-448]/16.
            for eye,partner,dd,od,mm,om,sign in [(q,p,dl,dr,lm,rm,1),(p,q,dr,dl,rm,lm,-1)]:
                z,valid,d=legacy.depths(dd,od,mm,om,sign)
                out[eye]=dict(depth=z,valid=valid,disparity=d,CAD=masks[eye],partner=partner,sign=sign)
        return out,time.monotonic()-begin
