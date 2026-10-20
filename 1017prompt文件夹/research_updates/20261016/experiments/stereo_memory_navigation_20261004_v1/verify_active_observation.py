from pathlib import Path
import json,datetime,hashlib
import numpy as np
from feature_tracks import extract,predict_features
from active_observation import guarded_predict,SafeFeatureMemory,admit_observation
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest();K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
class FutureTrap(dict):
    def __getitem__(self,key):
        if key!='time_s':raise AssertionError('future payload read')
        return super().__getitem__(key)
def main():
    p=json.loads((H/'active_observation_protocol.json').read_text());inf=json.loads((H/'active_feature_protocol.json').read_text());check={**p['source_sha256'],**p['previous_artifacts'],**inf['source_sha256']}
    for f,s in check.items():assert sha(f)==s,f
    r=H/'active_observation_run';o=H/'active_feature_runs';a=json.loads((o/'assessment.json').read_text());initial=json.loads((o/'assessment_initial16prefix.json').read_text());m=json.loads((o/'manifest.json').read_text());status=json.loads((H/'active_observation_status.json').read_text())
    assert p['sealed_utc']<status['start_utc']<status['end_utc']<inf['sealed_utc']<m['saved_utc']<a['assessed_utc'];assert status['exit_code']==0 and m['labels_not_loaded'];assert not a['source_candidate_accepted'] and a['motion']['gate_pass']
    for k in initial:
        if k not in ['prefix','assessed_utc']:assert initial[k]==a[k],k
    identity=json.loads((r/'identity.json').read_text());assert identity['completed'] and identity['requests']==180 and identity['captures']==181 and identity['reset_count']==1 and not identity['internal_reset'];assert identity['source_sha256']==p['source_sha256']
    commands=np.load(r/'requests.npz')['requests'];np.testing.assert_equal(commands,np.load(r/'planned_requests.npz')['requests']);assert commands.shape==(180,10) and not commands[:60].any()
    ph=np.load(r/'physics.npz');old=np.load(H/'online_bank_hold_navigation_run/physics.npz')
    fields=[k for k in ph.files if k not in ['raw_simulation_time_s','raw_physics_step_index']];assert len(fields)==17
    for k in fields:np.testing.assert_equal(ph[k][:301],old[k][:301])
    assert (ph['raw_simulation_time_s'][0],ph['raw_physics_step_index'][0])==(old['raw_simulation_time_s'][0],old['raw_physics_step_index'][0])
    np.testing.assert_equal(ph['raw_command_target'][1:],commands.reshape(900,2));assert len(ph['simulation_time_s'])==901
    states=json.loads((r/'execution_states.json').read_text());assert states[:61]==json.loads((H/'online_bank_hold_navigation_run/execution_states.json').read_text())[:61]
    e=next(e for e in json.loads((H/'guarded_response_library.json').read_text()) if e['name']=='positive');d=json.loads((r/'decision.json').read_text());grid=np.load(r/'offline_occupancy_label.npz');rebuilt=admit_observation(d['current'],d['execution'],e,grid['labels'],grid['origin'],float(grid['resolution']))
    for k,v in rebuilt.items():assert v==d[k],k
    assert d['admitted'] and d['floor_height_same'] and not d['full_3d_certified'];contacts=json.loads((r/'physics_contacts.json').read_text());assert len(contacts)==1081 and all(v['available'] and not v['contact'] for v in contacts)
    for markers in json.loads((r/'marker_checks.json').read_text()):assert all(v['computed_visibility']=='invisible' for v in markers.values())
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');memory=SafeFeatureMemory(10);history=[];previous=None;counts={arm:[0,0,0] for arm in ['current','causal']};future_checks=finite_parity=0
    for i in range(181):
        rgb=np.load(r/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(r/'known_poses'/f'frame_{i:03d}.npz');f=np.load(o/f'features_{i:03d}.npz');saved=json.loads((o/f'prediction_{i:03d}.json').read_text());row=m['rows'][i];assert i==row['frame'];assert sha(o/f'prediction_{i:03d}.json')==row['sha256'] and sha(o/f'features_{i:03d}.npz')==row['features_sha256']
        assert set(rgb.files)=={'left_rgb','right_rgb','time_s'} and set(pose.files)=={'left_camera_usd_world','right_camera_usd_world','time_s'};t=float(rgb['time_s']);assert np.isfinite(t) and t==float(pose['time_s']);pair=[]
        for eye in ['left','right']:
            xy,desc,size=extract(rgb[eye+'_rgb'],masks[eye]);pp=pose[eye+'_camera_usd_world'];assert np.isfinite(pp).all()
            for k,v in [('xy',xy),('desc',desc),('size',size),('pose',pp)]:np.testing.assert_equal(v,f[eye+'_'+k])
            pair.append(dict(frame=i,time_s=t,eye=eye,xy=xy,desc=desc,size=size,pose=pp))
        for arm,sources in [('current',[pair[1]]),('causal',[pair[1]]+history)]:
            pred=guarded_predict(pair[0],sources,K);assert pred==saved[arm];assert pred==predict_features(pair[0],sources,K);finite_parity+=1
            assert pred==guarded_predict(pair[0],sources+[FutureTrap(time_s=t+1)],K);future_checks+=1
            pts=[pt for pt in a['points'] if pt['frame']==i and pt['arm']==arm];assert len(pts)==len(pred['points']);counts[arm][0]+=pred['query_count'];counts[arm][1]+=len(pts);counts[arm][2]+=sum(pt['correct'] for pt in pts)
            for pt in pts:
                if pt['label_world'] is not None:assert np.linalg.norm(np.array(pt['world'])-pt['label_world'])==pt['error_m']
        memory.update(saved['causal']['points'],t,0.);assert memory.snapshot(t)==saved['memory'] and not memory.certified_free([0,0],t);history=(history+pair)[-6:]
        h=np.load(r/'history'/f'frame_{i:03d}.npz')
        if previous is not None:
            for k in h.files:np.testing.assert_equal(h[k][:-1],previous[k][1:])
        previous={k:h[k].copy() for k in h.files}
    for arm,c in counts.items():
        s=a['source_stats'][arm];assert c==[s['queries'],s['retained'],s['correct']] and not s['gate_pass'];assert s['correct']/s['queries']==s['unconditional_fraction']
    report=H.parent.parent/'reports/benchmark_original_scene_active_observation_20261005.md';files=list(r.rglob('*'))+list(o.rglob('*'))+list(H.glob('active_observation*'))+list(H.glob('active_feature_*'))+list((H/'figures').glob('active_observation*'))+[H/n for n in ['active_observation.py','test_active_observation.py','run_active_observation.py','run_active_observation_stage.py','run_active_features.py','assess_active_observation.py','assess_active_observation_initial16prefix.py','show_active_observation.py','write_active_observation_report.py','verify_active_observation.py']]+[report]
    files=[f for f in files if f.is_file() and f.name not in ['active_observation_verification.json','active_observation_verify_final.log']]
    result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=len(check),stage_tests=128,actual_sequences=1,actual_requests=180,captures=181,physics=901,contact_samples=1081,no_contact=True,actual_motion_gate=True,source_gate=False,whole_research_complete=False,feature_pairs_reextracted=181,predictions_reconstructed=362,finite_wrapper_parity_checks=finite_parity,future_payload_checks=future_checks,memory_snapshots=181,history_shift_checks=180,full_prefix_nonraw17_exact=True,raw_origin_same=True,source_and_artifact_sha256={str(f):sha(f) for f in files if f.exists()})
    (H/'active_observation_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='source_and_artifact_sha256'},indent=2))
if __name__=='__main__':main()
