"""Read-only author verification and unavailable-input decomposition, no flag regeneration."""
from common import *
from scipy.spatial import cKDTree
seal=load(D/'prediction_seal.json');cfg=load(D/'protocol.json');result=load(D/'result.json')
for path,h in cfg['generation_input_sha256'].items():assert sha(path)==h,path
for path,h in cfg['teacher_input_sha256'].items():assert sha(path)==h,path
with np.load(cfg['teacher_occupancy']) as occ:
    truth=cKDTree(np.argwhere(occ['grid']>0)*float(occ['resolution_m'])+occ['global_origin_xy'])
totals={};reason_names=['anchor_outside_axial_range','outside_native_FOV','CAD','matching_or_depth_invalid','usable'];reasons=np.zeros(5,np.int64);event_reasons=np.zeros(5,np.int64)
for row in seal['rows']:
    i=row['frame'];assert sha(row['advisory'])==row['sha256']
    with np.load(row['advisory']) as a,np.load(RUN/'predictions'/f'frame_{i:03d}.npz') as p:
        cells=a['cells'];bad=truth.query(cells*.1+a['origin'])[0]>.15
        # Independent Python reductions over cell -> list of saved anchor votes, no generation mutation.
        grouped=[[] for _ in cells]
        for k,j in enumerate(a['anchor_cell_index']):grouped[int(j)].append(bool(a['anchor_dispute'][k]))
        strict=np.array([bool(v) and all(v) and bool(a['historical'][j]) for j,v in enumerate(grouped)])
        broad=np.array([any(v) and bool(a['historical'][j]) for j,v in enumerate(grouped)])
        assert np.array_equal(strict,a['all_anchor_dispute']) and np.array_equal(broad,a['any_anchor_dispute'])
        for name,q in [('all_anchor_dispute',strict),('any_anchor_dispute',broad)]:
            for teacher,t in [('false',bad),('real',~bad)]:totals[name+'_'+teacher]=totals.get(name+'_'+teacher,0)+int((q&t).sum())
        for e,n in enumerate(NAMES):
            T=p['body_estimated']@LOCAL[e]
            # Explicit inverse homogeneous transform independent of production row-matrix operation.
            homogeneous=np.c_[a['anchor_world'],np.ones(len(a['anchor_world']))]
            xyz=(np.linalg.inv(T)@homogeneous.T).T[:,:3]@AX
            z=xyz[:,2];front=np.isfinite(xyz).all(1)&(z>=.5)&(z<=3)
            uv=np.zeros((len(xyz),2));uv[front]=xyz[front,:2]/z[front,None]*F+[319.5,239.5]
            fov=front&(uv[:,0]>=0)&(uv[:,0]<=639)&(uv[:,1]>=0)&(uv[:,1]<=479)
            u=np.rint(np.clip(uv[:,0],0,639)).astype(int);v=np.rint(np.clip(uv[:,1],0,479)).astype(int)
            cad=p[n+'_CAD'][v,u];dep=p[n+'_depth'][v,u];valid=p[n+'_valid'][v,u]&np.isfinite(dep)&(dep>=.5)&(dep<=3)
            reason=np.zeros(len(xyz),int);reason[front]=1;reason[fov]=2;reason[fov&~cad]=3;reason[fov&~cad&valid]=4
            counts=np.bincount(reason,minlength=5);reasons+=counts
            if i==12:event_reasons+=counts
            available=reason==4;rr=dep-z;vote=np.zeros(len(z),int);vote[available& (rr<-.2)]=1;vote[available&(abs(rr)<=.2)]=2;vote[available&(rr>.2)]=3
            assert np.array_equal(vote,a['current_votes'][:,e])
            assert np.array_equal((v*640+u)[available],a['current_query_uid'][:,e][available])
            assert np.allclose(z,a['anchor_current_z'][:,e],atol=1e-10,rtol=1e-10)
for k,v in totals.items():assert v==result['totals'][k],(k,v)
state=load(D/'status.json');assert state['status']=='finished' and state['child_returncode']==0
write(D/'author_audit.json',dict(complete=True,rows_verified=180,generation_identities_verified=len(cfg['generation_input_sha256']),teacher_identities_verified=len(cfg['teacher_input_sha256']),saved_vote_independent_homogeneous_inverse_exact=True,all_cell_reductions_exact=True,all_teacher_dispute_counts_exact=True,total_camera_anchor_queries=int(reasons.sum()),camera_anchor_input_reasons=dict(zip(reason_names,map(int,reasons))),first_event_camera_anchor_reasons=dict(zip(reason_names,map(int,event_reasons))),source_seal_sha256=sha(D/'prediction_seal.json'),audit_source_sha256=sha(Path(__file__)),no_output_regeneration=True,fit=0,physics=0))
print('AUDIT',dict(zip(reason_names,map(int,reasons))),flush=True)
