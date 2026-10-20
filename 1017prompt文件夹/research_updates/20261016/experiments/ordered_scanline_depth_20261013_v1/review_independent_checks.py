"""Independent review checks: binary exhaustive alignments and eligible shift costs."""
import itertools,json
import numpy as np
import torch
from joint import alignment,JointStereo
from test_joint import brute

def run():
    checked=0
    for vals in itertools.product(range(2),repeat=8):
        c=np.array(vals[:4],np.int32).reshape(2,2)
        gl=np.array(vals[4:6],np.int32);gr=np.array(vals[6:],np.int32)
        best,forced,possible,skip=brute(c,gl,gr)
        p,ok,num,sk,opt=[v.numpy()[0] for v in alignment(torch.from_numpy(c[None]),torch.from_numpy(gl[None]),torch.from_numpy(gr[None]))]
        assert int(opt)==best
        assert {(i,int(p[i])) for i in range(2) if ok[i]}==forced
        assert np.array_equal(num,possible) and np.array_equal(sk,skip)
        checked+=1
    rng=np.random.default_rng(814)
    a=rng.integers(0,256,(15,20),dtype=np.uint8);b=np.zeros_like(a);b[:,:-2]=a[:,2:]
    m=np.zeros_like(a,bool);c,_,_=JointStereo().costs(a,b,m,m);arr=c.numpy()
    eligible=sum(arr[y,x,x-2]==0 for y in range(4,11) for x in range(6,16))
    assert eligible==70
    return dict(binary_exhaustive_n2=checked,shifted_zero_cost_pairs=int(eligible),truth_read=False)
if __name__=='__main__':print(json.dumps(run()))
