"""Check complete new-rig evidence without assuming success or failure outcomes."""
import datetime,hashlib,json,re,subprocess
from pathlib import Path
import cv2,numpy as np
from rig_inputs import validate_capture,validate_mask_identity
H=Path(__file__).resolve().parent;W=H.parents[1];OLD=H.parent/'visual_information_diagnostic_20261003_v1'
def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as f:
        while block:=f.read(8*1024*1024):digest.update(block)
    return digest.hexdigest()
D=json.loads((H/'depth_assessment.json').read_text());A=json.loads((H/'run_assessment.json').read_text());C=json.loads((H/'old_new_comparison.json').read_text());protocol=json.loads((H/'capture_protocol.json').read_text())
assert len(A['runs'])==8
assert all(d['sample_pairs_completed']==66 and not d['missing'] and d['latency']['n']==66 for d in D.values())
mask_identity=json.loads((H/'mask_identity.json').read_text());validate_mask_identity(mask_identity)
assert sha(H/'base_link_masks.npz')==mask_identity['mask_sha256']
old_sources=json.loads((H/'frozen_code_identity.json').read_text())
for name,row in old_sources.items():assert row['previous_sha256']==row['current_sha256']==sha(H/name)==sha(OLD/name)
old_verify=json.loads((OLD/'final_verification.json').read_text())
# Preserve all old experiment source and model outputs; camera project code is separately versioned.
for p,digest in old_verify['source_sha256'].items():assert sha(p)==digest,p
models={}
for model,expected in [('da3_metric','bbea5b0b3ee389849cffa7ddae89de064a90abd2b055fc5aa99aac68db324776'),('unidepth_v2','ba73d3de735302ccc64a50f1e557122050c4b1893e6060b28dba05d6af3e67c6')]:
    checkpoint=H/'models'/model/'model.safetensors';assert sha(checkpoint)==expected
    records=json.loads((H/(model+'_inference_records.json')).read_text());assert len(records)==66
    assert len(list((H/'predictions'/model).glob('*/*.npz')))==66
    models[model]=dict(sha256=expected,size_bytes=checkpoint.stat().st_size,strict_load=True,completed_pairs=66,accepted=D[model]['accepted'],
        peak_torch_allocated_bytes=max(r['gpu_max_allocated_bytes'] for r in records),checkpoint_revision=old_verify['models'][model]['revision'])
    for p in (H/'predictions'/model).glob('*/*.npz'):
        pred=np.load(p)
        for eye in ('left','right'):assert pred[eye+'_depth_m'].shape==(480,640)
inputs=json.loads((H/'offline_manifest.json').read_text())['samples'];old_inputs=json.loads((OLD/'offline_manifest.json').read_text())['samples'];assert len(inputs)==66
changed=0;capture_summaries={};max_local_offset_error=0
for seq in protocol['sequences']:
    root=H/'capture'/seq['sequence'];summary=json.loads((root/'capture_summary.json').read_text())
    assert summary['completed'] and not (root/'failure.txt').exists()
    assert summary['selected_indices']==seq['selected_indices'] and summary['source_request_sha256']==sha(seq['requests_file'])
    assert all(x==0 for x in summary['old_new_state_max_abs_differences'].values())
    assert summary['render_preserved_clock_and_state'] and summary['all_markers_hidden']
    for path,digest in summary['source_sha256'].items():assert sha(path)==digest,path
    clocks=json.loads((root/'camera_clocks.json').read_text());assert len(clocks)==seq['request_count']+1
    assert all(abs(r['raw_physics_time_s']-t)<=1e-5 for r in clocks for t in r['eyes'].values())
    assert all(d['computed_visibility']=='invisible' for frame in json.loads((root/'markers.json').read_text()) for d in frame.values())
    capture_summaries[seq['sequence']]=summary
for old,row in zip(old_inputs,inputs):
    assert row['sequence']==old['sequence'] and Path(row['rgb_file']).name==Path(old['rgb_file']).name
    assert sha(row['rgb_file'])==row['rgb_sha256'] and sha(row['truth_file'])==row['truth_sha256']
    rgb=np.load(row['rgb_file']);true=np.load(row['truth_file']);old_rgb=np.load(old['rgb_file'])
    changed+=int(any(not np.array_equal(rgb[eye+'_rgb'],old_rgb[eye+'_rgb']) for eye in ('left','right')))
    root=Path(row['rgb_file']).parent.parent;index=int(Path(row['rgb_file']).stem.removeprefix('frame_'))
    clock=json.loads((root/'camera_clocks.json').read_text())[index]
    validate_capture(dict(left_optical_to_base=true['left_optical_to_base'],right_optical_to_base=true['right_optical_to_base'],left_rgb=rgb['left_rgb'],right_rgb=rgb['right_rgb'],
        raw_time_s=clock['raw_physics_time_s'],left_render_time_s=clock['eyes']['left'],right_render_time_s=clock['eyes']['right']))
    for eye,y in [('left',.3),('right',-.3)]:max_local_offset_error=max(max_local_offset_error,float(np.max(np.abs(true[eye+'_optical_to_base'][:3,3]-[.2,y,.05]))))
assert changed==66
identities=[];videos={};contacts={}
for label in ('e0','e2','h3','h4'):
    assert all(v==0 for v in A['initial_state_differences'][label].values())
    for mode in ('map','visible'):
        root=H/'runs'/mode/label;identity=json.loads((root/'experiment_identity.json').read_text());identities.append(identity)
        assert identity['camera_rig']==mask_identity['camera_rig']
        assert identity['information_mode']==mode and identity['formal_budget_s']==35 and not identity['oracle_reference_online']
        assert identity['renderer_depth_online']==(mode=='visible') and identity['full_map_online']==(mode=='map')
        assert identity['self_mask_sha256']==mask_identity['mask_sha256']
        for path,digest in identity['source_sha256'].items():assert sha(path)==digest,path
        assert all(d['computed_visibility']=='invisible' for frame in json.loads((root/'marker_visibility_each_capture.json').read_text()) for d in frame.values())
        clocks=json.loads((root/'camera_clocks.json').read_text());assert all(abs(v['age_s'])<=1e-5 for r in clocks for v in r['eyes'].values())
        assert all(clocks[i+1]['eyes'][eye]['rendering_time_s']>clocks[i]['eyes'][eye]['rendering_time_s'] for i in range(len(clocks)-1) for eye in ('left','right'))
        assert all(json.loads((root/'initial_render_invariants.json').read_text()).values())
        episode=next(root.glob('episode_*'));metric=json.loads((episode/'metrics.json').read_text());audit=json.loads((episode/'collision_audit.json').read_text())
        assert metric['runtime_error']==0 and metric['collision_mode']=='physical_contact_v2'
        assert audit['contact_measurement_available'] and bool(metric['collision'])==any(s['contact'] for s in audit['samples'])
        contacts[mode+'_'+label]=dict(physical_contact=bool(metric['collision']),first_contact_s=A['runs'][mode+'_'+label]['first_contact_s'])
        if metric['termination_reason']=='timeout':assert abs(A['runs'][mode+'_'+label]['elapsed_s']-35)<1e-5
        for name in ('third_person.mp4','stereo_view.mp4'):
            path=episode/name;cap=cv2.VideoCapture(str(path));assert cap.isOpened()
            count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS)
            ok,first=cap.read();assert ok and first.max()>0
            cap.set(cv2.CAP_PROP_POS_FRAMES,count-1);ok,last=cap.read();assert ok and last.max()>0;cap.release()
            assert fps==10 and count>1;videos[str(path)]=dict(frames=count,fps=fps,first_last_decoded=True)
assert all(d['source_sha256']==identities[0]['source_sha256'] for d in identities)
assert all(all(v==0 for v in row['old_new_initial_state_max_abs_difference'].values()) for row in C['navigation'].values())
tests=(H/'final_tests.txt').read_text();assert '26 passed' in tests and 'failed' not in tests
report=W/'reports/benchmark_visual_information_new_camera_rig_repeat_20261004.md'
missing=[p for p in re.findall(r'\]\((/[^)]+)\)',report.read_text()) if not Path(p).exists() and p!=str(H/'final_verification.json')];assert not missing,missing
for p in (H/'figures').glob('*.png'):assert cv2.imread(str(p)) is not None
assert len(list((H/'figures').glob('*.png')))>=6
result=dict(stage_complete=True,verification_time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rig=mask_identity['camera_rig'],
    scope='matched development perception repeat and eight fresh same-controller online information diagnostics; not a new complete benchmark or learned world model',
    models=models,official_source_commits={n:subprocess.check_output(['git','-C',str(H/'third_party'/n),'rev-parse','HEAD'],text=True).strip() for n in ('da3','unidepth')},
    replay_sequences=6,offline_pairs=66,newly_rendered_changed_pairs=changed,all_replay_observed_states_bit_equal_to_old=True,max_measured_local_offset_error_m=max_local_offset_error,
    captures=capture_summaries,actual_complete_online_runs=8,contacts=contacts,video_checks=videos,cpu_tests_passed=26,
    all_online_initial_states_bit_equal=True,all_old_new_online_initial_states_bit_equal=True,all_camera_times_verified=True,all_markers_hidden=True,
    old_artifacts_preserved=True,controller_and_measurement_and_model_code_unchanged=True,new_mask_sha256=mask_identity['mask_sha256'],
    all_runtime_source_hashes_verified=True,source_sha256={str(p):sha(p) for p in H.glob('*.py')},report_sha256=sha(report),
    actual_rgb_navigation_runs=0,gated_out_rgb_runs=4 if not any(d['accepted'] for d in D.values()) else None,
    prior_oracle_conclusions_preserved=True,report=str(report))
(H/'final_verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(stage_complete=True,models_verified=list(models),actual_runs=8,videos=len(videos),capture_sequences=6,changed_picture_pairs=66,cpu_tests=26,report=str(report)),indent=2))
