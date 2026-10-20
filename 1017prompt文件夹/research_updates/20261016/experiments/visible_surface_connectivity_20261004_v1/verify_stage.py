"""Evidence verification: actual artifacts, declared variants, identities and freshness."""
import ast,hashlib,json,re,datetime
from pathlib import Path
import numpy as np
import cv2
H=Path(__file__).resolve().parent;OLD=H.parent/'visual_information_new_rig_20261004_v2'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
status=json.loads((H/'run_status.json').read_text());assert len(status)==8 and all(x['exit_code']==0 and x['artifacts_complete'] for x in status)
r=json.loads((H/'run_assessment.json').read_text());assert all(all(x.values()) for x in r['paired_initial_state_equal'].values());assert all(all(x.values()) for x in r['historical_baseline_reproduction'].values())
versions={'points':H/'run_information_before_native_guard.py','surface':H/'run_information.py'}
helper_paths=(H/'surface_support.py',H/'surface_support_before_import_cleanup.py');helper_sha={sha(p):str(p) for p in helper_paths}
def function_asts(p):
    return [ast.dump(node,include_attributes=False) for node in ast.parse(p.read_text()).body if isinstance(node,(ast.FunctionDef,ast.ClassDef))]
assert function_asts(helper_paths[0])==function_asts(helper_paths[1]),'cleanup changed guard functions'
immutable={'rig_inputs.py','measurement.py','base_link_masks.npz','mask_identity.json'}
assert all(sha(H/name)==sha(OLD/name) for name in immutable)
identities=[];videos=[];counts=[]
for arm in ('points','surface'):
 for case in ('e0','e2','h3','h4'):
    root=H/'runs'/arm/case;identity=json.loads((root/'experiment_identity.json').read_text());assert identity['map_representation']==arm and identity['information_mode']=='visible'
    assert identity['camera_rig']['rig_id']=='ball_stereo_front020_up005_v2'
    assert identity['renderer_depth_online'] and not identity['full_map_online'] and not identity['oracle_reference_online']
    assert identity['formal_budget_s']==35 and identity['self_mask_sha256']==sha(H/'base_link_masks.npz')
    assert identity['roll_pid_gains']=={'kp':.4,'ki':.1,'kd':.7}
    for path,expected in identity['source_sha256'].items():
        file=Path(path)
        if file==H/'run_information.py':actual=sha(versions[arm])
        elif file==H/'surface_support.py':assert expected in helper_sha;actual=expected
        else:actual=sha(file)
        assert actual==expected,(path,arm,case)
    clocks=json.loads((root/'camera_clocks.json').read_text());assert len(clocks)==351
    for i,c in enumerate(clocks):
        assert all(abs(eye['age_s'])<1e-5 for eye in c['eyes'].values())
        if i:
            previous=clocks[i-1]
            assert all(c['eyes'][e]['rendering_frame']>previous['eyes'][e]['rendering_frame'] and c['eyes'][e]['rendering_time_s']>previous['eyes'][e]['rendering_time_s'] for e in ('left','right'))
    markers=json.loads((root/'marker_visibility_each_capture.json').read_text());assert len(markers)==351 and all(v['computed_visibility']=='invisible' for row in markers for v in row.values())
    inv=json.loads((root/'initial_render_invariants.json').read_text());assert inv['clock_unchanged'] and inv['physical_state_unchanged']
    inputs=list((root/'online_inputs').glob('*.npz'));truth=list((root/'offline_truth').glob('*.npz'));assert len(inputs)==350 and len(truth)==351
    snaps=list((root/'belief_snapshots').glob('*.npz'));assert len(snaps)==35
    ep=next(root.glob('episode_*'));metrics=json.loads((ep/'metrics.json').read_text());assert metrics['elapsed_s']==35 and metrics['episode_budget_s']==35 and metrics['evaluation_protocol_id']=='ball_pointgoal_v1_2_contact_35s'
    for file in ('third_person.mp4','stereo_view.mp4'):
        path=ep/file;cap=cv2.VideoCapture(str(path));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=float(cap.get(cv2.CAP_PROP_FPS));assert n>0 and fps==10
        ok,first=cap.read();assert ok and first.size
        cap.set(cv2.CAP_PROP_POS_FRAMES,n-1);ok,last=cap.read();assert ok and last.size;cap.release();videos.append(dict(path=str(path),frames=n,fps=fps,first_last_decode=True))
    counts.append(dict(arm=arm,case=case,pairs=len(clocks),action_blocks=len(inputs),truth_pairs=len(truth),belief_snapshots=len(snaps)))
    identities.append(dict(arm=arm,case=case,wrapper_source_sha256=sha(versions[arm]),helper_version_sha256=identity['source_sha256'].get(str(H/'surface_support.py')),identity_sha256=sha(root/'experiment_identity.json')))
# All control/protocol/supervisor dependencies are identical to original prior-stage runs.
for case in ('e0','e2','h3','h4'):
    prior=json.loads((OLD/'runs/visible'/case/'experiment_identity.json').read_text())['source_sha256']
    for arm in ('points','surface'):
        actual=json.loads((H/'runs'/arm/case/'experiment_identity.json').read_text())['source_sha256']
        for path,expected in prior.items():
            if path in actual:assert actual[path]==expected,(path,case,arm)
# Prior stage source identity and assessment remain frozen.
protocol=json.loads((H/'stage_protocol.json').read_text())
for name,expected in protocol['source_sha256'].items():assert sha(OLD/name)==expected
report=H.parents[1]/'reports/benchmark_visible_surface_connectivity_20261004.md';assert report.is_file()
for path in re.findall(r'\]\((/[^)]+)\)',report.read_text()):
    if Path(path)!=H/'final_verification.json':assert Path(path).exists(),path
assert len(list((H/'figures').glob('*.png')))>=5
cpu=(H/'cpu_tests.txt').read_text();assert '40 passed' in cpu and 'failed' not in cpu
summary=dict(stage_complete=True,time_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),actual_full_runs=8,camera_pairs_total=sum(v['pairs'] for v in counts),input_role='current actual stereo renderer depth; privileged diagnostic, not RGB navigation',candidate_accepted=r['candidate_accepted'],behavior_equivalence=r['behavior_equivalence'],cpu_tests=40,paired_initial_states_exact=True,historical_point_baseline_entire_requests_applied_and_xy_exact=True,native_guard_versions_have_identical_function_AST=True,identities=identities,counts=counts,videos=videos,report_sha256=sha(report),source_sha256={str(p):sha(p) for p in H.glob('*.py')},protocol_sha256=sha(H/'stage_protocol.json'),amendment_sha256=sha(H/'native_support_amendment.json'),provenance_clarification_sha256=sha(H/'provenance_clarification.json'))
(H/'final_verification.json').write_text(json.dumps(summary,indent=2)+'\n')
assert (H/'final_verification.json').is_file()
print(json.dumps({k:summary[k] for k in ('stage_complete','actual_full_runs','camera_pairs_total','candidate_accepted','cpu_tests','behavior_equivalence')},indent=2))
