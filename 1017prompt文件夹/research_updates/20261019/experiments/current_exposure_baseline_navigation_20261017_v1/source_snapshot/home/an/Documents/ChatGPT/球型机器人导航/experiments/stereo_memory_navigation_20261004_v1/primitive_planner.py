"""Measured-primitive 2.5D floor support check, including stopping displacement.

Only checks the floor/hazard representation. Positive support is explicitly NOT
a full3D collision certificate. This distinction prevents admission by map area.
"""
import numpy as np

def assess_support(xy_sequence,labels,origin,resolution,radius_m):
    path=np.asarray(xy_sequence,float).reshape(-1,2)
    if not len(path) or not np.isfinite(path).all():raise ValueError('finite whole trajectory required')
    labels=np.asarray(labels);origin=np.asarray(origin,float)
    # Densify measured trajectory so a long segment cannot skip a hazard cell.
    dense=[path[0]]
    for a,b in zip(path[:-1],path[1:]):
        n=max(1,int(np.ceil(np.linalg.norm(b-a)/(resolution/2))))
        dense.extend(a+(b-a)*np.arange(1,n+1)[:,None]/n)
    unknown=set();occupied=set();checked=set()
    r=radius_m+np.sqrt(2)*resolution/2
    for xy in dense:
        lo=np.floor((xy-r-origin)/resolution).astype(int)
        hi=np.ceil((xy+r-origin)/resolution).astype(int)
        for i in range(lo[0],hi[0]+1):
            for j in range(lo[1],hi[1]+1):
                if np.linalg.norm(origin+np.array([i,j])*resolution-xy)>r:continue
                key=(i,j);checked.add(key)
                if min(i,j)<0 or i>=labels.shape[0] or j>=labels.shape[1] or labels[i,j]<0:unknown.add(key)
                elif labels[i,j]>0:occupied.add(key)
    return dict(support_passed=not unknown and not occupied,unknown_cells=len(unknown),occupied_cells=len(occupied),
        checked_cells=len(checked),full_3d_certified=False,scope='2.5D observed floor support and recorded hazards only')
