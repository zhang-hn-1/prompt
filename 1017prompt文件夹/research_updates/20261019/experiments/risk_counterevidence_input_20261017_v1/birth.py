"""Teacher-only input intervention after P0 prediction seal. Never used in decisions."""
from common import *
from scipy.spatial import cKDTree

def main():
    cfg=load(D/'birth_protocol.json');assert load(RUN/'source_seal.json')['complete'] and not (D/'birth_result.json').exists()
    for f,h in cfg['input_sha256'].items():assert sha(f)==h,f
    with np.load(cfg['occupancy']) as occ:truth=cKDTree(np.argwhere(occ['grid']>0)*float(occ['resolution_m'])+occ['global_origin_xy'])
    seal=load(RUN/'source_seal.json');cap=load(RUN/'capture_seal.json');total=0;rows=[];tot={};error_lists={key:[] for key in ['false_depth_error','real_depth_error','false_depth_only_XY_shift','false_pose_only_XY_shift']};causes={};event_sources=[]
    with np.load(RUN/'maps/frame_000.npz') as m:origin=m['origin'].copy()
    with np.load(RUN/'maps/frame_061.npz') as m:
        cells=np.argwhere(m['labels']==1);bad=truth.query(cells*.1+origin)[0]>.15;event_false=set(map(tuple,cells[bad]))
    for i,(s,c) in enumerate(zip(seal['rows'],cap['rows'])):
        assert i==s['frame'];pf=RUN/'predictions'/c['frame'];ip=RUN/'frames'/c['frame']
        assert sha(pf)==s['prediction_sha256'] and sha(ip)==s['input_sha256'];stats={}
        with np.load(pf) as p,np.load(ip) as a:
            for e,n in enumerate(NAMES):
                uid=p[n+'_query_pixel'];world=p[n+'_world'];count=len(uid);total+=count
                if not count:continue
                uv=np.stack([(uid%640-319.5)/F,(uid//640-239.5)/F,np.ones(count)],1)
                z=p[n+'_depth'].reshape(-1)[uid];gt=a[n+'_depth_label'].reshape(-1)[uid];finite=np.isfinite(gt)&(gt>0)
                T=p['body_estimated']@LOCAL[e];G=a[n+'_camera_usd_world_label'];body_gt=G@np.linalg.inv(LOCAL[e])
                # No scene labels generated before source seal; fixed source UID domain throughout.
                true_world=(uv*gt[:,None])@(G[:3,:3]@AX).T+G[:3,3]
                depth_only=(uv*gt[:,None])@(T[:3,:3]@AX).T+T[:3,3]
                pose_only=(uv*z[:,None])@(G[:3,:3]@AX).T+G[:3,3]
                cells=np.floor((world[:,:2]-origin)/.1+.5).astype(int);false=truth.query(cells*.1+origin)[0]>.15
                stats['points']=stats.get('points',0)+count;stats['original_false_points']=stats.get('original_false_points',0)+int(false.sum())
                label=np.full(count,'other_height',dtype='<U24');label[~finite]='no_finite_GT'
                for name,condition in [('inside_body',finite&(np.linalg.norm(true_world-a['root_position_physics_label'],axis=1)<.55)),('floor',finite&(abs(true_world[:,2])<=.03)),('bodyband',finite&(true_world[:,2]>.05)&(true_world[:,2]<=1.5))]:label[condition]=name
                for name in np.unique(label):causes[name]=causes.get(name,0)+int((false&(label==name)).sum())
                error_lists['false_depth_error'].append(abs(z[false&finite]-gt[false&finite]));error_lists['real_depth_error'].append(abs(z[~false&finite]-gt[~false&finite]))
                error_lists['false_depth_only_XY_shift'].append(np.linalg.norm(world[false&finite,:2]-depth_only[false&finite,:2],axis=1))
                error_lists['false_pose_only_XY_shift'].append(np.linalg.norm(world[false,:2]-pose_only[false,:2],axis=1))
                for name,w,root,depth,ok in [('GT_depth_native_pose',depth_only,p['body_estimated'][:3,3],gt,finite),('pred_depth_GT_pose',pose_only,body_gt[:3,3],z,np.ones(count,bool)),('GT_depth_GT_pose',true_world,body_gt[:3,3],gt,finite)]:
                    eligible=ok&(depth>=.5)&(depth<=3)&(np.linalg.norm(w-root,axis=1)>=.55)&(w[:,2]>.05)&(w[:,2]<=1.5)
                    remaining_false=np.zeros(count,bool)
                    cc=np.floor((w[eligible,:2]-origin)/.1+.5).astype(int)
                    remaining_false[eligible]=truth.query(cc*.1+origin)[0]>.15
                    stats[name+'_remaining_original_false']=stats.get(name+'_remaining_original_false',0)+int((remaining_false&false).sum())
                    stats[name+'_excluded_original_false']=stats.get(name+'_excluded_original_false',0)+int((false&~eligible).sum())
                    stats[name+'_still_eligible_points']=stats.get(name+'_still_eligible_points',0)+int(eligible.sum())
                    stats[name+'_all_false_points']=stats.get(name+'_all_false_points',0)+int(remaining_false.sum())
                if i<=12:
                    selected=np.array([tuple(c) in event_false for c in cells])
                    for cell in np.unique(cells[selected],axis=0):
                        k=np.flatnonzero(selected&(cells==cell).all(1));j=k[np.argmin(uid[k])]
                        event_sources.append(dict(cell=cell.tolist(),frame=i,eye=n,query_uid=int(uid[j]),partner_uid=int(p[n+'_partner_pixel'][j]),point_count=len(k),pred_z_m=float(z[j]),GT_z_m=float(gt[j]) if finite[j] else None,teacher_surface=str(label[j]),world=world[j].tolist(),input_prediction_sha256=s['prediction_sha256']))
        rows.append(dict(frame=i,**stats))
        for k,v in stats.items():tot[k]=tot.get(k,0)+v
    assert total==cfg['expected_total_source_points']
    quant={}
    for key,values in error_lists.items():
        x=np.concatenate(values);quant[key]=dict(finite_points=len(x),P50_P90_P95_m=np.quantile(x,[.5,.9,.95]).tolist())
    false=tot['original_false_points'];depth_reduction=1-tot['GT_depth_native_pose_remaining_original_false']/false;pose_reduction=1-tot['pred_depth_GT_pose_remaining_original_false']/false
    write(D/'birth_result.json',dict(complete=True,source_frames=181,total_source_points=total,totals=tot,false_input_teacher_surface=causes,conditional_error_quantiles=quant,depth_intervention_false_input_reduction=depth_reduction,pose_intervention_false_input_reduction=pose_reduction,predeclared_depth_dominance_supported=depth_reduction>=.5 and pose_reduction<=.1,rows=rows,first_event_all_false_cell_source_lineage=event_sources,scope='All181 source frames including unused terminal; point-instance audit, not map navigation rescore. Original source UID selection held fixed. Teacher input interventions change coordinates and same source range/height/root gate at those UIDs. Exclusion becomes unknown in counterfactual only, never actual clearing. Floor/height teacher labels describe GT backprojection of original RGB query. Pose intervention uses render camera labels; packet phase and initial gauge remain disclosed.',source_regenerated=False,map_changed=False,navigation=0,fit=0,protocol_sha256=sha(D/'birth_protocol.json')))
    print('BIRTH',total,tot,quant,'DEPTH_REDUCTION',depth_reduction,'POSE_REDUCTION',pose_reduction,flush=True)
if __name__=='__main__':main()
