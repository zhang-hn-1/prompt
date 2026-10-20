"""Audit completed fixed-trajectory map artifacts without replaying physics."""
from pathlib import Path
import json
import hashlib
import numpy as np
from scipy.spatial import cKDTree

D=Path(__file__).resolve().parent
W=D.parents[1]
C=W/'experiments/source_confidence_calibration_20261012_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
source_review=json.loads((D/'independent_review.json').read_text())
report=json.loads((D/'map_results.json').read_text())
seal=json.loads((D/'map_prediction_seal.json').read_text())
assert all(sha(p)==h for p,h in seal['outputs'].items())
assert all(sha(p)==h for p,h in seal['source'].items())
assert seal['ownership_seal_sha']==sha(D/'prediction_seal.json')
assert all(sha(p)==h for p,h in seal['ownership_outputs'].items())
groups=[]
for ep,name in [('e0','E0'),('e1','E1')]:
    R=C/f'data_{ep}_run'
    cfg=json.loads((C/f'data_{ep}_protocol.json').read_text())
    with np.load(cfg['occupancy']) as occ:
        truth=occ['global_origin_xy']+np.argwhere(occ['grid']>0)*float(occ['resolution_m'])
    tree=cKDTree(truth)
    counts=np.zeros((2,3),dtype=np.int64)
    errors=[[],[]]
    paths=sorted((R/'partial_observations').glob('*.npz'))
    assert len(paths)==181
    for p in paths:
        with np.load(p) as a, np.load(D/'predictions'/f'{name}_{p.name}') as v, np.load(R/'frames'/p.name) as f:
            q=a['query_pixel_id']
            # Map admission has additional historical filters, and remains a subset of sensor candidates.
            assert np.isin(q,v['query']).all()
            keep=np.isin(q,v['query'][v['keep']])
            x,y=q%640,q//640
            gt=f['left_depth_m'][y,x]
            ok=np.isfinite(gt)&(gt>0)
            rays=np.stack([(x-319.5)/584.8888888888889,-(y-239.5)/584.8888888888889,-np.ones(len(q))],axis=1)
            T=f['left_camera_usd_world']
            points=T[:3,3]+(rays*np.where(ok,gt,0)[:,None])@T[:3,:3].T
            floor=ok&(np.abs(points[:,2])<=.03)
            for j,k in enumerate([np.ones(len(q),dtype=bool),keep]):
                counts[j]+=np.array([k.sum(),(k&ok).sum(),(k&floor).sum()])
                errors[j].extend(np.linalg.norm(a['world'][ok&k]-points[ok&k],axis=1))
    actual=next(r for r in report if r['sequence']==name)
    clocks=[]
    labelarrays=[]
    for j,arm in enumerate(['before','after']):
        with np.load(D/f'{name}_{arm}_map.npz') as m:
            cells=np.argwhere(m['labels']==1)
            xy=m['origin']+cells*float(m['resolution'])
            old=float(m['time_s'])-m['clock'][tuple(cells.T)]>10
            far=tree.query(xy)[0]>.15
            expected=dict(risk_cells=len(cells),older10s=int(old.sum()),far015=int(far.sum()),
                          older_far015=int((old&far).sum()),accepted_points=int(counts[j,0]),
                          finite_gt_points=int(counts[j,1]),gt_floor_false=int(counts[j,2]),
                          point_error_p50_p90=np.quantile(errors[j],[.5,.9]))
            row=next(r for r in actual['rows'] if r['arm']==arm)
            for key,value in expected.items():
                assert np.allclose(value,row[key],rtol=0,atol=1e-12),(name,arm,key)
            labelarrays.append(m['labels'].copy());clocks.append(float(m['time_s']))
    with np.load(R/'maps/frame_899.npz') as original:
        assert np.array_equal(labelarrays[0],original['labels'])
        assert clocks[0]==clocks[1]==float(original['time_s'])
    groups.append(dict(sequence=name,metrics_reproduced=True,all_admitted_points_subset_of_source=True,
                       final_baseline_labels_exact=True,final_map_time_s=clocks[0],
                       point_metric_frame_range='0..900 step5; full source run',
                       changed_final_label_cells=int(np.count_nonzero(labelarrays[0]!=labelarrays[1]))))
source_review['map_review']=dict(groups=groups,map_output_hashes_matched=len(seal['outputs']),
                               map_source_hashes_matched=len(seal['source']),
                               ownership_output_hashes_matched=len(seal['ownership_outputs']),
                               ownership_seal_binding_verified=True,
                               scope='fixed historical trajectories; no new physical run or navigation success',
                               notes=['Map snapshots end at frame899 (~89.9s); admitted point metrics include frame900 (~90s).',
                                      'Source candidate points and admitted map points are distinct populations.'],
                               reviewed_map_artifact_sha256={str(D/n):sha(D/n) for n in ['maps_and_visuals.py','map_results.json','map_prediction_seal.json']})
partitions=json.loads((D/'error_partition.json').read_text())
original_seal=json.loads((D/'prediction_seal.json').read_text())
source_results=json.loads((D/'result.json').read_text())
partition_checks=[]
for bound,reported in zip(original_seal['rows'],partitions['results']):
    root=Path(bound['directory']);name=bound['name']
    assert name==reported['sequence']
    labels=['ground_absz_le03','positive_obstacle_05_15','other_finite']
    totals=np.zeros((3,4),dtype=np.int64)
    for p in sorted((root/'source_predictions').glob('*.npz')):
        with np.load(p) as pred,np.load(D/'predictions'/f'{name}_{p.name}') as v, \
             np.load(root/'frames'/p.name) as f,np.load(root/'partial_observations'/p.name) as a:
            q=v['query'];keep=v['keep'];x,y=q%640,q//640
            gt=f['left_depth_m'][y,x];z=pred['depth_m'][y,x]
            ok=np.isfinite(gt)&(gt>0);safe=np.where(ok,gt,0)
            T=f['left_camera_usd_world']
            height=T[2,3]+safe*(T[2,0]*(x-319.5)/584.8888888888889-T[2,1]*(y-239.5)/584.8888888888889-T[2,2])
            floor=ok&(np.abs(height)<=.03);obstacle=ok&(height>.05)&(height<=1.5)
            admitted=np.isin(q,a['query_pixel_id']);bad=np.abs(z-gt)>.3
            for j,cls in enumerate([floor,obstacle,ok&~floor&~obstacle]):
                selected=keep&cls
                totals[j]+=[selected.sum(),(selected&bad).sum(),(selected&admitted).sum(),(selected&admitted&bad).sum()]
    for j,label in enumerate(labels):
        r=reported['partitions'][label]
        assert totals[j].tolist()==[r[k] for k in ['n','bad030','accepted','accepted_bad030']]
    baseline=next(r for r in source_results['results'] if r['sequence']==name)
    assert int(totals[:,0].sum())==baseline['finite_after']
    assert np.isclose(totals[:,1].sum()/totals[:,0].sum(),baseline['above030_fraction'],rtol=0,atol=1e-12)
    partition_checks.append(dict(sequence=name,all_partition_counts_reproduced=True,
                                 partition_total_matches_original_finite_score=True))
assert sha(D/'result.json')==source_review['audited_artifact_sha256'][str(D/'result.json')]
assert sha(D/'protocol.json')==source_review['audited_artifact_sha256'][str(D/'protocol.json')]
map_status=json.loads((D/'map_status.json').read_text())
interruption=json.loads((D/'map_interruption.json').read_text())
assert map_status['status']=='finished' and map_status['child_returncode']==0
assert interruption['shell_returncode']==143 and interruption['physics']==0
source_review['posthoc_partition_review']=dict(groups=partition_checks,
    original_result_and_protocol_hashes_unchanged=True,
    scope='GT-conditioned descriptive error origin only; not an online classifier, causal explanation, revised gate, or rescue',
    artifact_sha256={str(D/n):sha(D/n) for n in ['error_partition.py','error_partition.json']})
source_review['map_recovery_review']=dict(initial_shell_returncode=143,
    initial_interruption_is_not_performance_evidence=True,recovery_child_returncode=0,
    parent_pid=map_status['parent_pid'],child_pid=map_status['child_pid'],physics=0)
(D/'independent_review.json').write_text(json.dumps(source_review,indent=2)+'\n')
print(json.dumps(source_review['map_review'],indent=2))
print(json.dumps(source_review['posthoc_partition_review'],indent=2))
