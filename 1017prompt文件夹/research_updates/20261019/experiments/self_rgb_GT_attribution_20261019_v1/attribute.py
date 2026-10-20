"""One post-failure GT-only attribution of existing native perturbation outputs."""
from pathlib import Path
import json, hashlib, datetime
import numpy as np
D=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
put=lambda p,x:Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')

def clear_window(mask, ids):
    y,x=ids//640,ids%640
    inside=(y>=4)&(y<476)&(x>=3)&(x<637)
    integral=np.pad(mask.astype(np.int64),((1,0),(1,0))).cumsum(0).cumsum(1)
    lo_y=np.clip(y-4,0,480);hi_y=np.clip(y+5,0,480)
    lo_x=np.clip(x-3,0,640);hi_x=np.clip(x+4,0,640)
    count=integral[hi_y,hi_x]-integral[lo_y,hi_x]-integral[hi_y,lo_x]+integral[lo_y,lo_x]
    return inside&(count==0)

def group_stats(select, oldgood, newgood, newvalid):
    n=int(select.sum());gg=int((select&oldgood&newgood).sum());gb=int((select&oldgood&~newgood).sum())
    bg=int((select&~oldgood&newgood).sum());bb=int((select&~oldgood&~newgood).sum())
    assert gg+gb+bg+bb==n
    return dict(scorable=n,old_good=gg+gb,old_bad=bg+bb,still_good=gg,harmed=gb,corrected=bg,still_bad=bb,
                old_good_lost_validity=int((select&oldgood&~newvalid).sum()),new_valid=int((select&newvalid).sum()))

def main():
    cfg=json.loads((D/'protocol.json').read_text())
    for sec in ['input_sha256','code_sha256']:
        for f,h in cfg[sec].items():assert sha(f)==h,f
    old_result=json.loads(Path(cfg['native_dependency_result']).read_text())
    result_lookup={(r['sequence'],r['frame'],e['eye']):e for r in old_result['rows'] for e in r['eyes']}
    rows=[];artifacts=[];totals=dict(all_risk=0,protected=0,affected_all=0,affected_protected=0,full_pixels=0,finite_positive_GT=0,invalid_GT_UNKNOWN=0,target_pixels=0)
    aggregates={name:dict() for name in ['all_finite_in_range','protected_finite_in_range','affected_protected_finite_in_range','affected_protected_target']}
    F=294.77087816197377;Y,X=np.indices((480,640));rays=np.stack([(X-319.5)/F,(Y-239.5)/F,np.ones_like(X)],-1);AX=np.diag([1.,-1.,-1.])
    for row in cfg['rows']:
        with np.load(row['original_path']) as original,np.load(row['intervened_path']) as intervention,np.load(row['input_rgb_frame']) as raw:
            for eye in cfg['eyes']:
                partner=cfg['partners'][eye];q=original[eye+'_query_pixel'];p=original[eye+'_partner_pixel']
                z0=original[eye+'_depth'].ravel()[q].astype(float);z1=intervention[eye+'_depth'].ravel()[q].astype(float)
                v1=intervention[eye+'_valid'].ravel()[q];newvalid=v1&np.isfinite(z1)&(z1>=.5)&(z1<=3)
                affected=(~v1)|(np.abs(z0-z1)>.15)
                protected=clear_window(original[eye+'_CAD'],q)&clear_window(original[partner+'_CAD'],p)
                expected=result_lookup[row['sequence'],row['frame'],eye]
                assert len(q)==expected['old_risk_points'] and int(protected.sum())==expected['protected_risk_points']
                assert np.array_equal(q[affected&protected],np.asarray(expected['protected_changed_query_pixel']))
                # First numerical GT read: original/perturbed outputs were sealed in the earlier independent experiment.
                z=raw[eye+'_depth_label'];ref=z.ravel()[q].astype(float)
                finite=np.isfinite(ref)&(ref>0);inrange=finite&(ref>=.5)&(ref<=3)
                T=raw[eye+'_camera_usd_world_label'];root=raw['root_position_physics_label']
                with np.errstate(invalid='ignore'):
                    world=(rays*z[...,None])@(T[:3,:3]@AX).T+T[:3,3]
                target=np.isfinite(z)&(z>=.5)&(z<=3)&~original[eye+'_CAD']&(np.linalg.norm(world-root,axis=2)>=.55)&(world[...,2]>.05)&(world[...,2]<=1.5)
                error0=np.abs(z0-ref);error1=np.abs(z1-ref)
                oldgood=inrange&(error0<=.15);newgood=inrange&newvalid&(error1<=.15)
                selectors=dict(all_finite_in_range=inrange,protected_finite_in_range=protected&inrange,
                    affected_protected_finite_in_range=affected&protected&inrange,affected_protected_target=affected&protected&target.ravel()[q])
                stats={name:group_stats(sel,oldgood,newgood,newvalid) for name,sel in selectors.items()}
                for name,m in stats.items():
                    for k,v in m.items():aggregates[name][k]=aggregates[name].get(k,0)+v
                counts=dict(all_risk=len(q),protected=int(protected.sum()),affected_all=int(affected.sum()),affected_protected=int((affected&protected).sum()),
                    full_pixels=z.size,finite_positive_GT=int((np.isfinite(z)&(z>0)).sum()),invalid_GT_UNKNOWN=int((~(np.isfinite(z)&(z>0))).sum()),target_pixels=int(target.sum()))
                for k,v in counts.items():totals[k]+=int(v)
                attr=affected&protected
                domain=dict(affected_protected_invalid_GT=int((attr&~finite).sum()),affected_protected_positive_GT_outside_range=int((attr&finite&~inrange).sum()),affected_protected_in_range=int((attr&inrange).sum()),affected_protected_in_target=int((attr&target.ravel()[q]).sum()))
                assert sum(domain[k] for k in ['affected_protected_invalid_GT','affected_protected_positive_GT_outside_range','affected_protected_in_range'])==counts['affected_protected']
                path=D/(row['sequence']+'_'+str(row['frame']).zfill(3)+'_'+eye+'.npz')
                np.savez_compressed(path,query_pixel=q,partner_pixel=p,protected=protected,affected=affected,GT_depth_m=ref,GT_finite_in_range=inrange,GT_in_target=target.ravel()[q],original_depth_m=z0,intervened_depth_m=z1,intervened_valid_for_reference=newvalid,original_good_within15cm=oldgood,intervened_good_within15cm=newgood)
                artifacts.append(dict(path=str(path),sha256=sha(path)));rows.append(dict(sequence=row['sequence'],frame=row['frame'],eye=eye,counts=counts,affected_GT_domain=domain,groups=stats))
    assert totals['all_risk']==12267 and totals['protected']==12052 and totals['affected_protected']==866
    assert totals['full_pixels']==4915200 and totals['target_pixels']==527344
    target=aggregates['affected_protected_target'];gates=dict(affected_target_nonempty=target['scorable']>0,
        original_error_majority=target['old_bad']>target['old_good'],corrections_exceed_harm=target['corrected']>target['harmed'])
    for sec in ['input_sha256','code_sha256']:
        for f,h in cfg[sec].items():assert sha(f)==h,f
    put(D/'attribution_seal.json',dict(complete=True,outputs=artifacts,GT_read=True,source_calls=0,fit=0,physics=0,navigation=0,protocol_sha256=sha(D/'protocol.json')))
    put(D/'result.json',dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),totals=totals,groups=aggregates,rows=rows,
        continuation_gates=gates,decision='NECESSARY_DIAGNOSTIC_CONTINUATION_CONDITIONS_HOLD_NOT_A_REPAIR' if all(gates.values()) else 'STOP_TREATING_SELF_RGB_DEPENDENCE_AS_UNIFORM_ERROR_SIGNAL',
        same_v123_repair_rejection_unchanged=True,original_source_gate_unchanged=True,source_calls=0,matcher_calls=0,new_predictions=0,fit=0,physics=0,navigation=0,
        scope='Postfailure GT-only attribution of frozen nuisance intervention, not online input/method/performance candidate. Axial±.15m reference agreement is not radial/grid/semantic/physical truth. Nonfinite/out-of-range GT remains unscored UNKNOWN. EditedRGB is not deployed. No threshold/CAD/decoder or old budget change.'))
if __name__=='__main__':main()
