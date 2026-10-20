import numpy as np
import pytest
from spatial_cost import aggregate_cost,decode_evidence

DIRECTIONS=[(dy,dx) for dy in (-1,0,1) for dx in (-1,0,1) if dy or dx]

def literal(cost,p1,p2):
    d,h,w=cost.shape;total=np.zeros_like(cost,dtype=float)
    for dy,dx in DIRECTIONS:
        path=np.full_like(cost,np.inf,dtype=float)
        for y in (range(h) if dy>=0 else range(h-1,-1,-1)):
            for x in (range(w) if dx>=0 else range(w-1,-1,-1)):
                yy,xx=y-dy,x-dx
                if yy<0 or yy>=h or xx<0 or xx>=w or not np.isfinite(path[:,yy,xx]).any():
                    path[:,y,x]=cost[:,y,x];continue
                prev=path[:,yy,xx];lo=np.min(prev)
                for i in range(d):
                    options=[prev[j]+(0 if i==j else p1 if abs(i-j)==1 else p2) for j in range(d)]
                    path[i,y,x]=cost[i,y,x]+min(options)-lo
        total+=path
    return total/8

def test_eight_paths_match_literal_unequal_costs_and_mask_barrier():
    c=np.random.default_rng(10).uniform(0,2,(5,4,6)).astype(np.float32);c[:,2,2]=np.inf;c[1,0,0]=np.inf
    np.testing.assert_allclose(aggregate_cost(c),literal(c,.05,.20),atol=1e-6)

def test_neighbours_change_isolated_ambiguous_winner_without_changing_input():
    c=np.ones((3,7,7),np.float32);c[1]=0;c[0,3,3]=0;c[1,3,3]=.02;before=c.copy()
    assert np.argmin(c[:,3,3])==0
    assert np.argmin(aggregate_cost(c)[:,3,3])==1
    np.testing.assert_array_equal(c,before)

def test_zero_penalties_equal_unary_and_all_invalid_stays_invalid():
    c=np.random.default_rng(1).uniform(0,2,(4,5,6)).astype(np.float32);c[:,:,3]=np.inf
    np.testing.assert_allclose(aggregate_cost(c,0,0),c,atol=1e-6)
    assert np.isinf(aggregate_cost(c)[:,:,3]).all()

def test_invalid_barrier_restarts_without_dependency_on_other_side():
    c=np.ones((3,1,5),np.float32);c[:,:,2]=np.inf;c[1,:,3:]=0
    changed=c.copy();changed[:,:,:2]=100
    np.testing.assert_array_equal(aggregate_cost(c)[:,:,3:],aggregate_cost(changed)[:,:,3:])

def test_symmetric_transpose_and_reverse():
    c=np.random.default_rng(8).uniform(0,2,(5,4,6)).astype(np.float32)
    a=aggregate_cost(c)
    np.testing.assert_allclose(aggregate_cost(c.transpose(0,2,1)).transpose(0,2,1),a,atol=1e-6)
    np.testing.assert_allclose(aggregate_cost(c[:,::-1,::-1])[:,::-1,::-1],a,atol=1e-6)

@pytest.mark.parametrize('p1,p2',[(.3,.2),(-1,.2),(float('nan'),.2),(.05,float('inf'))])
def test_invalid_penalties_raise(p1,p2):
    with pytest.raises(ValueError):aggregate_cost(np.zeros((3,2,2)),p1,p2)

def evidence(c,reg=None):
    return decode_evidence(np.asarray(c,np.float32)[:,None,None],np.ones((len(c),1,1),np.uint8),np.ones((1,1)),
        np.array([1.9,2.,2.1]),np.asarray(c if reg is None else reg,np.float32)[:,None,None])

def test_spatial_prior_cannot_certify_locally_invalid_bad_or_ambiguous_labels():
    for c,reg in [([1,np.inf,0],[1,0,1]),([.0,.4,.8],[1,0,1]),([0.,0.,0.],[1,0,1]),([np.inf]*3,[0,1,2])]:
        assert not evidence(c,reg)['confidence'].any()

def test_qualified_literal_depth_keeps_cost_provenance_and_halfcell_interval():
    o=evidence([.8,0.,.8]);assert o['confidence'].item()
    assert o['depth_m'].item()==2.;assert o['view_bits'].item()==1
    assert abs(o['interval_ray_width_m'].item()-.1)<1e-6

def test_competitive_regularized_winner_above_original_minimum_is_explicit():
    o=evidence([.8,.04,.0],[1,0,1])
    assert o['winner_index'].item()==1
    assert not o['confidence'].item() # interval includes original endpoint minimum
    assert o['unary_best_index'].item()==2
