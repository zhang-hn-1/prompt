"""Fixed saved-data diagnostic. No runtime actor, CAD, sampling, or model calls."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def distance(points,line):
    points=np.asarray(points,float).reshape(-1,2);line=np.asarray(line,float).reshape(-1,2)
    if not len(points) or not len(line):return None
    if len(line)==1:return np.linalg.norm(points-line[0],axis=1)
    d=np.diff(line,axis=0);s=(d*d).sum(1)
    valid=s>1e-24;d=d[valid];start=line[:-1][valid];s=s[valid]
    if not len(d):return np.linalg.norm(points-line[0],axis=1)
    u=np.clip(((points[:,None]-start)*d).sum(2)/s,0,1)
    return np.linalg.norm(points[:,None]-(start+u[:,:,None]*d),axis=2).min(1)
def summary(cells,origin,res,lines):
    centers=origin+np.asarray(sorted(cells),float).reshape(-1,2)*res
    result={'cells':len(cells),'cell_indices':sorted(cells),'world_centers':centers.tolist()}
    for name,line in lines.items():
        v=distance(centers,line)
        result[name]=None if v is None else {'minimum_m':float(v.min()),'within_original_guard_0p6':int((v<.6-1e-8).sum()),'within_logged_planning_margin':int((v<.8707106781186548-1e-8).sum())}
    return result
if __name__=='__main__':
    design=H/'dense_current_live_corridor_design_v1.json';prior=H/'dense_current_live_consumer_transfer_diagnostic_v1.json'
    spec=json.loads(design.read_text());inputs=json.loads(prior.read_text())['inputs_sha256']
    for p,s in inputs.items():assert sha(p)==s,p
    roots={a:H/f'dense_current_live_e1_{a}_run_v1' for a in ['b0','candidate']}
    for root in roots.values():
        seal=json.loads((root/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
        for p,s in inputs.items():
            if Path(p).parent==root or root in Path(p).parents:assert seal[p]==s,p
    rows={a:json.loads((r/'policy_records.json').read_text()) for a,r in roots.items()};out=[]
    for f in spec['fixed_epochs']:
        maps={a:np.load(r/'maps'/f'frame_{f:03d}.npz') for a,r in roots.items()}
        b,c=maps['b0'],maps['candidate'];assert np.array_equal(b['origin'],c['origin']) and float(b['resolution'])==float(c['resolution'])
        origin=c['origin'];res=float(c['resolution']);risk={a:set(map(tuple,np.argwhere(z['labels']==1).tolist())) for a,z in maps.items()}
        cr=rows['candidate'][f];sf=cr['source_frame'];p=np.load(roots['candidate']/'partial_observations'/f'frame_{sf:03d}.npz')
        q=p['query_pixel_id'];cells=np.rint((p['world'][:,:2]-origin)/res).astype(int);inside=((cells>=0)&(cells<np.array(c['labels'].shape))).all(1);even=((q%640)%2==0)&((q//640)%2==0)
        full=set(map(tuple,cells[inside].tolist()));sparse=set(map(tuple,cells[inside&even].tolist()));assert len(full-sparse)==next(r['extra_current_cells'] for r in json.loads(prior.read_text())['epochs'] if r['frame']==f)
        ref=cr.get('installed_reference_xy_after_act') or [];dec=cr.get('reference_decision');g=(dec or {}).get('guard')
        lines={'installed_reference':ref}
        if g:lines.update(suffix=g['suffix_xy'],bridge=g['bridge_xy'])
        out.append({'frame':f,'source_frame':sf,'reference_version':cr.get('local_reference_version'),'decision':(dec or {}).get('decision'),'logged_guard':g,'same_image_extra_current':summary(full-sparse,origin,res,lines),'candidate_only_actual_map':summary(risk['candidate']-risk['b0'],origin,res,lines),'b0_only_actual_map':summary(risk['b0']-risk['candidate'],origin,res,lines),'both_refs_exact':np.array_equal(np.array(ref),np.array(rows['b0'][f].get('installed_reference_xy_after_act') or []))})
    result={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'code_sha256':sha(__file__),'design_sha256':sha(design),'prior_diagnostic_sha256':sha(prior),'input_sha256':inputs,'epochs':out,'new_source_queries':0,'new_physics_runs':0,'not_online_validation':True,'not_full_causal_claim':True,'no_risk_cleared':True,'whole_mission_complete':False}
    target=H/'dense_current_live_corridor_diagnostic_v2.json';assert not target.exists();target.write_text(json.dumps(result,indent=2)+'\n')
    for r in out:print(r['frame'],r['decision'],{k:r[k] for k in ['same_image_extra_current','candidate_only_actual_map','b0_only_actual_map']})
