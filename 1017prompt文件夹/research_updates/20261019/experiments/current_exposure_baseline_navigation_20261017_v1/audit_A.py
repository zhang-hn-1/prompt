"""Postrun full actual A source-clock/current-only map and artifact audit, no navigation rerun."""
from paths import *
import json,numpy as np,hashlib,cv2
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
out=D/'e1_run';receipt=json.loads((D/'evaluation_receipt.json').read_text());manifest=json.loads((out/'artifact_manifest_post_close.json').read_text())
for f,h in manifest['artifact_sha256'].items():assert sha(f)==h,f
cap=json.loads((out/'capture_seal.json').read_text());source=json.loads((out/'source_seal.json').read_text());policy=json.loads((out/'policy_records.json').read_text());native=json.loads((out/'native_imu.json').read_text());result=json.loads((D/'e1_result.json').read_text());sources=source['rows'];assert len(cap['rows'])==len(sources)==result['frames'];assert len(policy)==result['actions'];expected={};counts=[]
for i,(s,c) in enumerate(zip(sources,cap['rows'])):
    assert s['frame']==i and sha(out/'predictions'/c['frame'])==s['prediction_sha256'] and sha(out/'frames'/c['frame'])==c['sha256']==s['input_sha256']
    assert native[c['imu_index']]['qualified'] and abs(native[c['imu_index']]['time_s']-s['time_s'])<1e-7
    with np.load(out/'predictions'/c['frame']) as p:
        assert abs(float(p['time_s'])-s['time_s'])<1e-8
        world=np.concatenate([p[n+'_world'] for n in ['front_left','front_right','rear_left','rear_right']]);expected[i]=world;counts.append(len(world))
assert sum(counts)==result['total_risk_points'];assert sum(r['target_cells'] for r in result['source_rows'])==result['source']['target_cells'];assert sum(r['covered_cells'] for r in result['source_rows'])==result['source']['covered_cells']
max_clock=0.;map_cells=0
for i,row in enumerate(policy):
    sf=int(row['source_frame']);st=float(row['source_time_s']);assert st<=row['time_s']+1e-8 and abs(st-sources[sf]['time_s'])<1e-8
    with np.load(out/'maps'/f'frame_{i:03d}.npz') as m:
        assert abs(float(m['time_s'])-row['time_s'])<1e-8
        world=expected[sf];cell=np.rint((world[:,:2]-m['origin'])/float(m['resolution'])).astype(int);cell=cell[(cell>=0).all(1)&(cell<np.array(m['labels'].shape)).all(1)];cell=np.unique(cell,axis=0);actual=np.argwhere(m['labels']==1)
        assert np.array_equal(cell,actual),(i,len(cell),len(actual))
        assert not np.any(m['labels']==0)
        if len(actual):max_clock=max(max_clock,float(abs(m['hazard_time'][tuple(actual.T)]-st).max()))
        map_cells+=len(actual)
assert max_clock<1e-8
videos={}
for n in ['front_left','front_right','rear_left','rear_right']:
    v=cv2.VideoCapture(str(out/(n+'.mp4')));count=0
    while True:
        ok,_=v.read()
        if not ok:break
        count+=1
    v.release();assert count==len(sources),(n,count);videos[n]=count
with np.load(out/'physics.npz') as ph:assert len(ph['simulation_time_s'])==len(native);physics=len(ph['simulation_time_s'])
B=W/'experiments/four_camera_weak_baseline_20261017_v1';a=json.loads((out/'loaded_source_identity.json').read_text());b=json.loads((B/'e1_run/loaded_source_identity.json').read_text());shared=set(a)&set(b)
for p in shared:assert a[p]==b[p],p
(D/'author_audit.json').write_text(json.dumps(dict(complete=True,manifest_files=len(manifest['artifact_sha256']),manifest_sha256=sha(out/'artifact_manifest_post_close.json'),source_frames=len(sources),source_points=sum(counts),policy_maps=len(policy),all_A_map_risk_equals_latest_actual_source_risk=True,all_unknown_never_observed_free=True,all_risk_acquisition_clock_matches_latest_source=True,map_risk_cell_queries=map_cells,decoded_video_frames=videos,native_IMU_physics_samples=physics,shared_A_B_loaded_module_bytes_equal=len(shared),source_copy_bitwise=sha(D/'source.py')==sha(B/'source.py'),no_GT_decision_or_new_inference_run=True),indent=2)+'\n')
print('AUTHOR_AUDIT_A',len(sources),len(policy),physics,videos,len(shared),flush=True)
