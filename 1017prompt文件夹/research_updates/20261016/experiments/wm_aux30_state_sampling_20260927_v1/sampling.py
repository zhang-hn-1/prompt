"""Fixed train-only actual-state stratification; not a runtime policy."""
import numpy as np

def classify_anchor(xy_speed,forward_speed,aux_deg):
    if xy_speed>=.15 and forward_speed<-.05:
        if aux_deg>=15:return 1
        if aux_deg<=-15:return 2
    return 0

def balanced_schedule(groups,updates,batch_size,seed):
    if set(groups)!={0,1,2} or any(not x for x in groups.values()):raise ValueError('Three nonempty groups required')
    flat=[i for g in groups.values() for i in g]
    if len(flat)!=len(set(flat)):raise ValueError('Groups must be disjoint and unique')
    n=updates*batch_size
    if updates<1 or batch_size<1 or n%20:raise ValueError('Positive budget divisible by20 required')
    rng=np.random.RandomState(seed);labels=np.repeat(np.arange(3),[n*18//20,n//20,n//20]);rng.shuffle(labels);out=np.empty(n,dtype=np.int64)
    for group in [0,1,2]:
        places=np.flatnonzero(labels==group);values=[]
        while len(values)<len(places):values.extend(rng.permutation(groups[group]).tolist())
        out[places]=values[:len(places)]
    return out.reshape(updates,batch_size)
