"""Independent small exhaustive alignments and direct local costs, without GT."""
import json,itertools,time
import numpy as np
import torch
from joint import alignment,JointStereo

def brute(c,gl,gr):
    n=len(gl);paths=[]
    def go(i,j,value,pairs):
        if i==j==n:paths.append((value,frozenset(pairs)));return
        if i<n:go(i+1,j,value+gl[i],pairs)
        if j<n:go(i,j+1,value+gr[j],pairs)
        if i<n and j<n:go(i+1,j+1,value+int(c[i,j]),pairs+[(i,j)])
    go(0,0,0,[]);best=min(p[0] for p in paths);opt=[p[1] for p in paths if p[0]==best]
    forced=set.intersection(*map(set,opt));possible=[len({j for p in opt for i,j in p if i==x}) for x in range(n)]
    skip=[any(not any(i==x for i,j in p) for p in opt) for x in range(n)]
    return best,forced,possible,skip

def main():
    rng=np.random.default_rng(20261013);checked=0
    for n in [1,2,3,4]:
        for case in range(12):
            c=rng.integers(0,8,(n,n),dtype=np.int32);gl=rng.integers(0,4,n,dtype=np.int32);gr=rng.integers(0,4,n,dtype=np.int32)
            best,forced,possible,skip=brute(c,gl,gr)
            out=alignment(torch.from_numpy(c[None]),torch.from_numpy(gl[None]),torch.from_numpy(gr[None]))
            p,ok,np_,sk,value=[v.numpy()[0] for v in out]
            assert int(value)==best
            assert {(i,int(p[i])) for i in range(n) if ok[i]}==forced
            assert np.array_equal(np_,possible) and np.array_equal(sk,skip)
            checked+=1
    # Explicit equal alternatives are unavailable, unique ordering path is retained.
    c=np.full((1,3,3),1000,np.int32);c[0,1,0]=0;c[0,2,1]=0
    p,ok,_,_,_=alignment(torch.from_numpy(c),torch.full((1,3),54),torch.full((1,3),54))
    assert ok.tolist()==[[False,True,True]] and p[0,1:].tolist()==[0,1]
    a=rng.integers(0,256,(15,20),dtype=np.uint8);b=rng.integers(0,256,a.shape,dtype=np.uint8);mask=np.zeros(a.shape,bool)
    model=JointStereo();cost,gl,gr=model.costs(a,b,mask,mask);arr=cost.numpy();direct=0
    for y in range(4,11):
        for xl in range(4,16):
            for xr in range(4,xl):
                expected=0
                for dy,dx in itertools.product(range(-1,2),repeat=2):
                    for cy,cx in itertools.product(range(-3,4),repeat=2):
                        if cy==cx==0:continue
                        expected+=((a[y+dy+cy,xl+dx+cx]<a[y+dy,xl+dx])!=(b[y+dy+cy,xr+dx+cx]<b[y+dy,xr+dx]))
                assert arr[y,xl,xr]==(expected if expected<=108 else 30000)
                direct+=1
    out=model.match_rectified(np.zeros_like(a),np.zeros_like(b),mask,mask)
    assert not out['valid'].any(),'constant appearance has alternative optimum assignments'
    return dict(exhaustive_cases=checked,exact_ordered_case=True,direct_local_costs=direct,constant_unknown=True)

if __name__=='__main__':print(json.dumps(main()))
