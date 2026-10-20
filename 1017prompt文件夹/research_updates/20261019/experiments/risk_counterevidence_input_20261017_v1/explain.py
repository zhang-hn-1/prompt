"""Postseal exhaustive partition; explains frozen flags, never changes them."""
from common import *
from scipy.spatial import cKDTree
cfg=load(D/'protocol.json');seal=load(D/'prediction_seal.json')
with np.load(cfg['teacher_occupancy']) as occ:truth=cKDTree(np.argwhere(occ['grid']>0)*float(occ['resolution_m'])+occ['global_origin_xy'])
names=['strict_dispute','some_farther_but_not_all_anchor_dispute','no_farther_with_compatible_pair','no_farther_with_closer_pair','only_partial_mixed_pairs','all_pairs_unavailable']
counts=np.zeros((2,6),int);events=[]
for row in seal['rows']:
    assert sha(row['advisory'])==row['sha256']
    with np.load(row['advisory']) as a:
        bad=truth.query(a['cells']*.1+a['origin'])[0]>.15;n=len(bad)
        has=np.zeros((n,5),bool)
        for code in range(5):np.logical_or.at(has[:,code],a['anchor_cell_index'],(a['current_pair_classes']==code).any(1))
        category=np.full(n,5,int);category[has[:,4]]=4;category[has[:,1]]=3;category[has[:,2]]=2;category[has[:,3]]=1;category[a['all_anchor_dispute']]=0
        for k in range(6):
            for t in range(2):counts[t,k]+=int((a['historical']&(bad==bool(t))&(category==k)).sum())
        if row['frame']==12:
            for k in np.flatnonzero(bad):
                ak=np.flatnonzero(a['anchor_cell_index']==k)
                events.append(dict(cell=a['cells'][k].tolist(),current_positive=bool(a['current_positive'][k]),reason=names[category[k]] if a['historical'][k] else 'current_positive_protected',anchor_frames=a['anchor_frame'][ak].tolist(),anchor_eyes=a['anchor_eye'][ak].tolist(),pair_codes=a['current_pair_classes'][ak].tolist()))
r=load(D/'result.json');assert counts[1].sum()==r['totals']['historical_false'] and counts[0].sum()==r['totals']['historical_real']
write(D/'cause_partition.json',dict(complete=True,false_historical_partition=dict(zip(names,map(int,counts[1]))),real_historical_partition=dict(zip(names,map(int,counts[0]))),first_event_all15_false_cells=events,teacher_selection_only_postseal=True,advisory_changed=False,scope='Mutually exclusive cell-query explanation, prioritizes strict, any paired farther, compatible, closer, mixed, unavailable. Not a new policy or a clearance certificate.'))
print('PARTITION',dict(zip(names,map(int,counts[1]))),flush=True)
