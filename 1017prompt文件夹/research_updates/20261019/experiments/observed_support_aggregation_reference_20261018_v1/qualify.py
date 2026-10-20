"""Necessary interface/causality checks only, no project test suite/real source."""
from pathlib import Path
import json,hashlib,itertools,datetime
import numpy as np
from kernel import aggregate,decode
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
put=lambda p,v:Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def guard(cfg):
 for section in ['code_sha256','input_sha256']:
  for p,h in cfg[section].items():assert sha(p)==h,p

def enum_path(c,m,coords,P1,P2):
    # Enumerate every path assignment since last unsupported node. No recurrence.
    if not m[coords[-1]].any():return np.full(c.shape[-1],np.inf),0
    start=0
    for i,q in enumerate(coords[:-1]):
        if not m[q].any():start=i+1
    coords=coords[start:];choices=[np.flatnonzero(m[q]).tolist() for q in coords]
    out=np.full(c.shape[-1],np.inf);count=0
    for ds in itertools.product(*choices):
        v=sum(float(c[q][di]) for q,di in zip(coords,ds))
        for a,b in zip(ds,ds[1:]):v+=0 if a==b else P1 if abs(a-b)==1 else P2
        out[ds[-1]]=min(out[ds[-1]],v);count+=1
    return out,count

def compare_brute(c,m,total,paths):
    H,W,N=c.shape; brute={k:np.full(c.shape,np.inf) for k in paths};enumerated=0
    for y in range(H):
        for x in range(W):
            for name,coords in [('left',[(y,a) for a in range(x+1)]),('right',[(y,a) for a in range(W-1,x-1,-1)]),('top',[(a,x) for a in range(y+1)])]:
                b,ct=enum_path(c,m,coords,600,2400);enumerated+=ct;brute[name][y,x]=b
                a=paths[name][y,x];finite=np.isfinite(a)
                assert np.array_equal(finite,np.isfinite(b))
                if finite.any():assert np.allclose(a[finite]-a[finite].min(),b[finite]-b[finite].min(),rtol=0,atol=1e-12)
    bsum=sum(brute.values())
    for y in range(H):
        for x in range(W):
            valid=np.isfinite(total[y,x]);assert np.array_equal(valid,np.isfinite(bsum[y,x]))
            if valid.any():assert np.allclose(total[y,x,valid]-total[y,x,valid].min(),bsum[y,x,valid]-bsum[y,x,valid].min(),rtol=0,atol=1e-12)
    assert np.array_equal(decode(total),decode(bsum));return enumerated

def main():
    cfg=json.loads((D/'protocol.json').read_text());guard(cfg);saved={};checks={};rows=[];calls=0;enumerated=0
    c=np.tile(np.array([0.,20.,1000.,2000.]),(2,5,1));m=np.ones(c.shape,bool);m[:,:,0]=False
    for fill in [0.,100000.]:
        cc=c.copy();cc[~m]=fill
        for kind,mm in [('unmasked',np.ones_like(m)),('masked',m)]:
            before=cc.tobytes(),mm.tobytes();a,p=aggregate(cc,mm);calls+=1;w=decode(a)
            assert before==(cc.tobytes(),mm.tobytes());enumerated+=compare_brute(cc,mm,a,p)
            key=kind+'_'+str(int(fill));saved[key+'_aggregate']=a;saved[key+'_winner']=w;rows.append(dict(case=key,winners=w.tolist()))
    checks['masked_invalid_fill_bitwise_invariant']=np.array_equal(saved['masked_0_aggregate'],saved['masked_100000_aggregate'])
    checks['masked_unique_supported_index1']=bool(np.all(saved['masked_0_winner']==1))
    checks['unmasked_artificial_context_changes_winner']=bool(np.all(saved['unmasked_0_winner']==0) and np.all(saved['unmasked_100000_winner']==1))
    fixtures=[]
    fixtures.append(('allmissing',np.full((2,3,3),np.nan),np.zeros((2,3,3),bool)))
    fixtures.append(('tie',np.array([[[0.,20.,20.]]]),np.array([[[False,True,True]]])))
    fixtures.append(('gap',np.array([[[1000.,0.,1000.],[np.nan,np.nan,np.nan],[1000.,1000.,0.]]]),np.array([[[True,True,True],[False,False,False],[True,True,True]]])))
    rng=np.random.default_rng(20261018)
    for i in range(8):fixtures.append(('seeded'+str(i),rng.integers(0,4001,size=(2,4,3)).astype(float),rng.random((2,4,3))>.25))
    for name,c,m in fixtures:
        cb=c.tobytes();mb=m.tobytes();a,p=aggregate(c,m);calls+=1;w=decode(a)
        assert cb==c.tobytes() and mb==m.tobytes();enumerated+=compare_brute(c,m,a,p)
        saved[name+'_aggregate']=a;saved[name+'_winner']=w;saved[name+'_cost']=c;saved[name+'_support']=m
        checks[name+'_no_invalid_winner']=bool(all(idx<0 or m[y,x,idx] for y,x in np.ndindex(w.shape) for idx in [int(w[y,x])]))
        if name in ['allmissing','tie']:checks[name+'_UNKNOWN']=bool(np.all(w==-1))
        if name=='gap':checks['gap_restart_no_free']=np.array_equal(w,np.array([[1,-1,2]],np.int32))
    checks['all_input_arrays_immutable']=True;checks['all_paths_match_exhaustive']=True
    # GT-free existence witness in current declared 480-wide rotated image.
    x=450;partner=322;d=x-partner;rad=3;z=294.77087816197377*.65012/d
    checks['single_hypothesis_support_can_keep_original_Z_domain']=(rad<=x<=476 and rad<=partner<=476 and .5<=z<=3 and d==128)
    try:aggregate(np.array([[[np.nan]]]),np.ones((1,1,1),bool));checks['invalid_supported_cost_rejected']=False
    except ValueError:checks['invalid_supported_cost_rejected']=True
    assert all(checks.values()),checks
    np.savez_compressed(D/'reference_arrays.npz',**saved)
    put(D/'result.json',dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),complete=True,decision='SMALL_MASKED_AGGREGATION_CONTRACT_PASSED_NOT_RGB_SOURCE',checks=checks,reference_aggregate_calls=calls,total_aggregate_entry_calls=calls+1,rejected_interface_calls=1,full_path_assignments_enumerated=enumerated,synthetic_context_rows=rows,single_hypothesis_witness=dict(query_x=x,partner_x=partner,disparity_px=d,Z_m=z),saved_arrays_sha256=sha(D/'reference_arrays.npz'),scope='Synthetic handcost and seededtinycost arrays only; no observed RGB data, BT cost computation, actual4eye depth, GT, spatialmap, full SGBM equivalence or performance/latency evidence.',RGB_source_calls=0,fit=0,physics=0,navigation=0))
    guard(cfg);print('ALL_NECESSARY_CHECKS',len(checks),'AGGREGATE_CALLS',calls,'ENUMERATED',enumerated,flush=True)
if __name__=='__main__':main()
