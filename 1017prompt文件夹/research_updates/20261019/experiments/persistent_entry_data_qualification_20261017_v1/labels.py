"""Offline complete static bodyband cell teacher AFTER feature seal. Never online free."""
from common import *
from scipy.spatial import cKDTree

def main():
    cfg=load(D/'protocol.json');seal=load(D/'feature_seal.json');assert seal['complete'] and not seal['teacher_read'];assert not (D/'qualification_result.json').exists()
    for p,h in cfg['teacher_input_sha256'].items():assert sha(p)==h,p
    teachers={}
    for group,path in cfg['teacher_occupancy'].items():
        with np.load(path) as occ:
            origin=occ['global_origin_xy'].copy();res=float(occ['resolution_m']);shape=occ['grid'].shape;assert abs(res-.1)<1e-8
            xy=np.argwhere(occ['grid']>0)*res+origin;teachers[group]=(cKDTree(xy),origin,res,np.array(shape))
    groups={g:dict(rows=0,eligible=0,positive=0,negative=0,unknown=0,eligible_positive=0,eligible_negative=0,eligible_unknown=0,positive_cells=set(),negative_cells=set(),exposure_frames=0,nonempty_frames=0) for g in teachers};seqstats={};rows=[];out=D/'labels';out.mkdir()
    eye_counts={g:{n:{'positive':0,'negative':0,'unknown':0} for n in NAMES} for g in teachers}
    for row in seal['rows']:
        f=Path(row['feature_path']);assert sha(f)==row['feature_sha256'];tree,origin,res,shape=teachers[row['group']]
        with np.load(f) as a:
            cells=a['cells'];xy=cells*.1+a['origin'];teacher_cell=np.rint((xy-origin)/res).astype(int);inside=(teacher_cell>=0).all(1)&(teacher_cell<shape).all(1)
            distance=tree.query(xy)[0] if len(xy) else np.empty(0);y=np.full(len(cells),-1,np.int8)
            y[inside&(distance<=.10+1e-12)]=1;y[inside&(distance>=.20-1e-12)]=0
            eligible=a['promotion_eligible'];assert a['features'].shape==(len(y),len(FEATURES)) and np.isfinite(a['features']).all()
            g=groups[row['group']];g['rows']+=len(y);g['eligible']+=int(eligible.sum());g['exposure_frames']+=1;g['nonempty_frames']+=int(len(y)>0)
            rr=dict(sequence=row['sequence'],frame=row['frame'],time_s=row['time_s'],rows=len(y),eligible=int(eligible.sum()))
            for label,name in [(1,'positive'),(0,'negative'),(-1,'unknown')]:
                g[name]+=int((y==label).sum());g['eligible_'+name]+=int(((y==label)&eligible).sum());rr[name]=int((y==label).sum());rr['eligible_'+name]=int(((y==label)&eligible).sum())
                if label>=0:g[name+'_cells'].update(map(tuple,teacher_cell[(y==label)&eligible].tolist()))
            for e,n in enumerate(NAMES):
                touched=np.unique(a['source_cell_row'][a['source_eye']==e]);target=y[touched]
                for label,name in [(1,'positive'),(0,'negative'),(-1,'unknown')]:eye_counts[row['group']][n][name]+=int((target==label).sum())
            dest=out/(row['sequence']+'_'+f.name);np.savez_compressed(dest,label=y,teacher_distance_m=distance,teacher_inside_domain=inside,promotion_eligible=eligible,cells=cells,time_s=row['time_s'])
            rr.update(label_file=str(dest),sha256=sha(dest),feature_sha256=row['feature_sha256']);rows.append(rr)
            s=seqstats.setdefault(row['sequence'],dict(frames=0,rows=0,eligible=0,positive=0,negative=0,unknown=0,eligible_positive=0,eligible_negative=0,eligible_unknown=0));s['frames']+=1
            for k in list(s):
                if k!='frames':s[k]+=rr[k]
    gates={}
    for name,g in groups.items():
        for label in ['positive','negative']:g['unique_eligible_'+label+'_cells']=len(g.pop(label+'_cells'))
        known=g['eligible_positive']+g['eligible_negative'];g['eligible_known_label_fraction']=known/g['eligible'] if g['eligible'] else None
        gates[name]=dict(known_eligible_labels_ge90pct=g['eligible_known_label_fraction'] is not None and g['eligible_known_label_fraction']>=.9,eligible_positive_instances_ge50=g['eligible_positive']>=50,eligible_negative_instances_ge50=g['eligible_negative']>=50,unique_positive_cells_ge20=g['unique_eligible_positive_cells']>=20,unique_negative_cells_ge20=g['unique_eligible_negative_cells']>=20,nonempty_source_frames_ge11=g['nonempty_frames']>=11)
    globalgates=dict(complete470_exposures=len(rows)==470,finite24_feature_channels=True,feature_then_teacher_separation=True,all_admit_mock_matches290_B_maps=seal['all_admit_mock_B_maps_exact']==290,current_recent_causal_contract=all(seal['checks'].values()),E1_A_B_same_scene_one_group=True,no_fresh_holdout_claim=True)
    passed=all(globalgates.values()) and all(all(g.values()) for g in gates.values())
    result=dict(complete=True,decision='DEVELOPMENT_DATA_AND_CONSUMER_INTERFACE_QUALIFIED_NOT_FITTED' if passed else 'NOT_QUALIFIED_BEFORE_FIT',qualified=passed,groups=groups,sequences=seqstats,per_eye_cell_exposure_counts=eye_counts,group_gates=gates,global_gates=globalgates,rows=rows,original_source_points=seal['original_source_points'],outside_consumer_grid_points=seal['outside_consumer_grid_points'],label_scope='Frozen fullbodyband occupancy teacher at proposed .1mgridcell. positive within.10m ofstatic occupiedcell; negative>=.20m; intermediate/outside teachergrid UNKNOWN. Training margin only; original .15m evaluation unchanged. Not GTquerysemanticheight or online measuredfree; fullphysicaltruth is itself a coarse2D benchmark proxy.',split='E0 train-development group; E1B andE1A ONE development scene group, not independent heldouts. All seen. Fresh heldout contract/budget must be frozen before actualfit; no freshdata claimed.',consumer='Recent10s allsource positive retained; only Btwoepoch persistententry requires futurefrozen score>=.95. NaN/missing score no promotion, never freespace. Mock1 equality is engineering only, NOT learned predictionperformance.',old_overlap='Static cellrisk learning and RGB/historyfeatures existed before. Structural intervention is persistententry ONLY with invariant recent source layer; not claiming a new statictruth concept or reviving oldweights/currentpositivefilter.',fit=0,learned_inference=0,source_regeneration=0,physics=0,navigation=0,full3arm_complete=False,reliable_navigation=False,learning_increment=False,feature_seal_sha256=sha(D/'feature_seal.json'),protocol_sha256=sha(D/'protocol.json'))
    write(D/'label_seal.json',dict(complete=True,rows=rows,feature_seal_sha256=sha(D/'feature_seal.json'),teacher_input_sha256=cfg['teacher_input_sha256']))
    write(D/'qualification_result.json',result);print('QUALIFICATION',result['decision'],groups,gates,globalgates,flush=True)
if __name__=='__main__':main()
