"""Verify artifact completeness and identities before claiming stage completion."""
import json,hashlib,subprocess,re,datetime,importlib.metadata
from pathlib import Path
import numpy as np
import cv2
H=Path(__file__).resolve().parent;W=H.parents[1]
def sha(path):
    d=hashlib.sha256()
    with Path(path).open('rb') as f:
        while block:=f.read(8*1024*1024):d.update(block)
    return d.hexdigest()
D=json.loads((H/'depth_assessment.json').read_text());A=json.loads((H/'run_assessment.json').read_text())
assert len(A['runs'])==8 and all(d['sample_pairs_completed']==66 and not d['missing'] and not d['accepted'] and d['latency']['n']==66 for d in D.values())
models={}
for model,name,revision,expected in [('da3_metric','DA3METRIC-LARGE','4010e39f3634a45bc60553321fb49fb760bd594e','bbea5b0b3ee389849cffa7ddae89de064a90abd2b055fc5aa99aac68db324776'),('unidepth_v2','unidepth-v2-vitl14','52b349b514bd8b47642f67ac78cb7b5dc5c51dd9','ba73d3de735302ccc64a50f1e557122050c4b1893e6060b28dba05d6af3e67c6')]:
    f=H/'models'/model/'model.safetensors';digest=sha(f);assert digest==expected
    assert len(list((H/'predictions'/model).glob('*/*.npz')))==66
    models[model]=dict(name=name,revision=revision,sha256=digest,size_bytes=f.stat().st_size,strict_load=True,completed_pairs=66)
    records=json.loads((H/(model+'_inference_records.json')).read_text());assert len(records)==66
    models[model]['peak_torch_allocated_bytes']=max(r['gpu_max_allocated_bytes'] for r in records)
    for f in (H/'predictions'/model).glob('*/*.npz'):
        p=np.load(f)
        for eye in ('left','right'):assert p[eye+'_depth_m'].shape==(480,640)
inputs=json.loads((H/'offline_manifest.json').read_text())['samples']
assert all(sha(row['rgb_file'])==row['rgb_sha256'] for row in inputs)
identities=[];videos={};contacts={}
for label in ('e0','e2','h3','h4'):
    assert all(d==0 for d in A['initial_state_differences'][label].values())
    for mode in ('map','visible'):
        root=H/'runs'/mode/label;identity=json.loads((root/'experiment_identity.json').read_text());identities.append(identity)
        assert identity['information_mode']==mode and identity['formal_budget_s']==35 and not identity['oracle_reference_online']
        assert identity['renderer_depth_online']==(mode=='visible') and identity['full_map_online']==(mode=='map')
        assert all(sha(path)==digest for path,digest in identity['source_sha256'].items())
        assert all(d['computed_visibility']=='invisible' for frame in json.loads((root/'marker_visibility_each_capture.json').read_text()) for d in frame.values())
        clocks=json.loads((root/'camera_clocks.json').read_text())
        assert all(v['age_s']==0 for row in clocks for v in row['eyes'].values())
        assert all(clocks[i+1]['eyes'][eye]['rendering_time_s']>clocks[i]['eyes'][eye]['rendering_time_s'] for i in range(len(clocks)-1) for eye in ('left','right'))
        assert all(json.loads((root/'initial_render_invariants.json').read_text()).values())
        episode=next(root.glob('episode_*'));metric=json.loads((episode/'metrics.json').read_text());audit=json.loads((episode/'collision_audit.json').read_text())
        assert metric['runtime_error']==0 and metric['collision_mode']=='physical_contact_v2'
        assert audit['contact_measurement_available'] and bool(metric['collision'])==any(d['contact'] for d in audit['samples'])
        contacts[mode+'_'+label]=dict(physical_contact=bool(metric['collision']),physical_contact_threshold_n=audit['contact_threshold_n'])
        for name in ('third_person.mp4','stereo_view.mp4'):
            path=episode/name;cap=cv2.VideoCapture(str(path));assert cap.isOpened()
            count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS);w=cap.get(cv2.CAP_PROP_FRAME_WIDTH);h=cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            ok,first=cap.read();assert ok and np.max(first)>0
            cap.set(cv2.CAP_PROP_POS_FRAMES,count-1);ok,last=cap.read();assert ok and np.max(last)>0;cap.release()
            assert fps==10 and count>1
            videos[str(path)]=dict(frames=count,fps=fps,width=w,height=h,first_last_decoded=True)
assert all(d['source_sha256']==identities[0]['source_sha256'] for d in identities)
tests=(H/'final_tests.txt').read_text();assert '22 passed' in tests and 'failed' not in tests
report=W/'reports/benchmark_visual_information_and_metric_depth_diagnostic_20261004.md'
links=re.findall(r'\]\((/[^)]+)\)',report.read_text())
missing=[p for p in links if not Path(p).exists() and p!=str(H/'final_verification.json')];assert not missing,missing
for p in (H/'figures').glob('*.png'):assert cv2.imread(str(p)) is not None
gpu=subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader'],text=True).strip()
official={name:subprocess.check_output(['git','-C',str(H/'third_party'/name),'rev-parse','HEAD'],text=True).strip() for name in ('da3','unidepth')}
result=dict(stage_complete=True,scope='two frozen offline models and eight actual same-controller information diagnostics; RGB4 gated out; no accepted new visual navigation baseline',
    verification_time=datetime.datetime.now(datetime.timezone.utc).isoformat(),models=models,official_source_commits=official,gpu=gpu,
    packages={p:importlib.metadata.version(p) for p in ('torch','torchvision','timm','transformers','huggingface-hub','numpy','opencv-python')},
    vendor_packages={d.metadata['Name']:d.version for d in importlib.metadata.distributions(path=[str(H/'vendor')])},source_sha256={str(p):sha(p) for p in H.glob('*.py')},protocol_sha256={str(p):sha(p) for p in (H/'PROTOCOL.md',H/'CONTROL_DIAGNOSTIC.md')},
    report_sha256=sha(report),offline_input_pairs=66,all_input_rgb_hashes_unchanged=True,actual_complete_runs=8,gated_out_rgb_runs=4,initial_observed_state_differences=A['initial_state_differences'],
    all_runtime_source_identities_equal=True,all_runtime_source_hashes_unchanged=True,all_camera_ages_zero=True,all_markers_hidden=True,all_render_refresh_state_invariants=True,
    contacts=contacts,video_checks=videos,cpu_tests_passed=22,fresh_final_review='no P1/P2; two minor report wording/layout findings corrected',unperformed=['RGB-depth navigation','new braking calibration','V-current/V-memory24','X-NavDP adaptation','world model training'],
    no_pid_rig_planner_baseline_modification=True,existing_oracle_conclusions_preserved=True)
(H/'final_verification.json').write_text(json.dumps(result,indent=2)+'\n')
assert all(Path(p).exists() for p in links)
print(json.dumps(dict(completed=result['stage_complete'],models_verified=list(models),runs=8,videos=len(videos),tests=22,report=str(report),gpu=gpu),indent=2))
