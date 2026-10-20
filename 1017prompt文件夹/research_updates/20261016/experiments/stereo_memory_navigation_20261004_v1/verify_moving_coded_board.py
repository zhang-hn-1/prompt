"""Author verification; independent saved-array reconstruction, no simulator/predictor rerun."""
import json,hashlib,datetime
from pathlib import Path
import cv2,numpy as np
from observation_schedule import schedule
H=Path(__file__).resolve().parent;W=H.parent.parent;o=H/'moving_coded_board_run'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'moving_coded_board_protocol.json').read_text());old_checks=0
for f,s in {**p['source_sha256'],**p['previous_artifacts']}.items():assert sha(f)==s,f;old_checks+=1
status=json.loads((H/'moving_coded_board_status.json').read_text());assert status['exit_code']==0
pre=json.loads((H/'moving_coded_board_startup_failure_status.json').read_text());assert pre['exit_code']==-11
assert len(list((H/'moving_coded_board_startup_failure/frames').iterdir()))==0
identity=json.loads((o/'identity.json').read_text());assert (identity['actions'],identity['captures'],identity['saved_frames'],identity['reset_count'],identity['recorded_video_frames'])==(120,121,6,1,121)
for f,s in identity['source_sha256'].items():assert sha(f)==s,f
requests=np.load(o/'requests.npz')['requests'];np.testing.assert_equal(requests,schedule('forward'))
phy=np.load(o/'physics.npz');original=np.load(H/'motion_render_ablation_probe/physics.npz');assert len(phy['simulation_time_s'])==601
same=[k for k in phy.files if k not in ['raw_simulation_time_s','raw_physics_step_index']]
assert len(same)==17
for k in same:np.testing.assert_equal(phy[k],original[k])
np.testing.assert_allclose(phy['raw_simulation_time_s']-original['raw_simulation_time_s'],-.02,atol=1e-8,rtol=0)
np.testing.assert_equal(phy['raw_physics_step_index']-original['raw_physics_step_index'],-1)
np.testing.assert_allclose(phy['raw_command_target'][1:],requests.reshape(-1,2),atol=1e-7,rtol=0)
clocks=json.loads((o/'camera_clocks.json').read_text());assert len(clocks)==121
ages=[]
for i,row in enumerate(clocks):
    assert row['frame']==i and row['markers_hidden']
    for value in row['eyes'].values():ages.append(abs(value-row['raw_time_s']))
assert max(ages)<=1e-5
for i in range(120):
    a=np.load(o/'history'/f'frame_{i:03d}.npz');b=np.load(o/'history'/f'frame_{i+1:03d}.npz')
    for k in a.files:np.testing.assert_equal(a[k][1:],b[k][:-1])
for name in ['stereo_view','third_person']:
    cap=cv2.VideoCapture(str(o/(name+'.mp4')));assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT))==121 and cap.get(cv2.CAP_PROP_FPS)==10.;cap.release()
geometry=json.loads((o/'board_geometry.json').read_text());assert not geometry['collision'] and geometry['subdivision']=='none';assert geometry['canonical_corner_permutation']==[1,0,3,2]
assess=json.loads((o/'geometry_assessment.json').read_text());manifest=json.loads((o/'rgb_detections/manifest.json').read_text());states=json.loads((o/'states.json').read_text());masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz')
assert status['start_utc']>p['sealed_utc'] and status['end_utc']<assess['detections_saved_utc']<assess['labels_evaluated_utc']
assert manifest['geometry_not_loaded'] and manifest['rgb_only']
for r in manifest['rows']:assert sha(r['path'])==r['sha256']
expected=recovered=correct=0
for row in assess['rows']:
    i=row['frame'];eye=row['eye'];pose=np.array(states[i][eye+'_pose']);R=pose[:3,:3]@np.diag([1.,-1.,-1.]);past=np.array(states[max(i-1,0)][eye+'_pose']);Rp=past[:3,:3]@np.diag([1.,-1.,-1.])
    d=np.load(o/'rgb_detections'/f'frame_{i:03d}_{eye}.npz');lookup={int(tag):j for j,tag in enumerate(d['ids'])};errors=[];olderrors=[];eligible=[]
    for tag,centre in enumerate(geometry['centres']):
        # Hand-written canonical marker offsets: -Y columns / -Z rows; no production geometry helper.
        points=np.array(centre)+np.array([[0,.14,.14],[0,-.14,.14],[0,-.14,-.14],[0,.14,-.14]])
        optical=np.linalg.solve(R,(points-pose[:3,3]).T).T
        xy=optical[:,:2]/optical[:,2,None]*584.8888888888889+np.array([319.5,239.5])
        if not (np.isfinite(xy).all() and (optical[:,2]>0).all() and (xy[:,0]>=2).all() and (xy[:,0]<638).all() and (xy[:,1]>=2).all() and (xy[:,1]<478).all()):continue
        poly=np.zeros((480,640),np.uint8);cv2.fillConvexPoly(poly,np.rint(xy).astype(int),1)
        if np.any(poly.astype(bool)&masks[eye]) or cv2.contourArea(xy.astype(np.float32))<400 or np.linalg.norm(xy-np.roll(xy,1,axis=0),axis=1).min()<20:continue
        eligible.append(tag)
        if tag in lookup:
            e=np.linalg.norm(d['corners'][lookup[tag]]-xy,axis=1);errors.extend(e)
            op=np.linalg.solve(Rp,(points-past[:3,3]).T).T;pxy=op[:,:2]/op[:,2,None]*584.8888888888889+np.array([319.5,239.5]);olderrors.extend(np.linalg.norm(d['corners'][lookup[tag]]-pxy,axis=1))
    assert eligible==row['eligible_tags']
    n=4*len(eligible);got=len(errors);good=int((np.array(errors)<=2).sum())
    assert (n,got,good)==(row['expected_corners'],row['recovered_corners'],row['correct_corners_2px'])
    np.testing.assert_allclose([np.sqrt(np.mean(np.square(errors))),np.percentile(errors,90),max(errors),np.percentile(olderrors,90)],[row['current_rms_px'],row['current_p90_px'],row['current_peak_px'],row['past_p90_px']],atol=1e-10,rtol=0)
    independent_gate=n>=12 and got>=.9*n and good>=.9*n and np.sqrt(np.mean(np.square(errors)))<=1.5 and np.percentile(errors,90)<=2
    assert row['gate_pass']==independent_gate
    expected+=n;recovered+=got;correct+=good
assert (expected,recovered,correct)==(420,356,356) and not assess['geometry_gate_pass']
summary=json.loads((o/'summary.json').read_text());assert not summary['matching_run']
assert summary['missing_tags']==16 and summary['missing_true_quad_decodable']==0
assert all(not r['whole_tag_overlap'] for r in summary['current_whole_tag_overlap'])
label=json.loads((o/'missing_photometry_labels.json').read_text())['rows'];assert len(label)==16 and all(r['white_border_cells']>2 and 'border' in r['rejected_by'] for r in label)
assert p['matching_budget']['max_anchors_per_frame']*p['matching_budget']['max_frames']==72 # Defect retained, not silently repaired.
oldp=json.loads((H/'moving_coded_board_protocol_preflight_v1.json').read_text())
for source,snapshot in [('coded_board.py','coded_board_preflight_v1.py'),('test_coded_board.py','test_coded_board_preflight_v1.txt'),('run_moving_coded_board.py','run_moving_coded_board_preflight_v1.py'),('audit_moving_coded_board.py','audit_moving_coded_board_preflight_v1.py')]:assert sha(H/snapshot)==oldp['source_sha256'][str(H/source)]
files=[f for f in o.rglob('*') if f.is_file()]+[f for f in (H/'moving_coded_board_startup_failure').rglob('*') if f.is_file()]
files += [H/x for x in ['coded_board.py','test_coded_board.py','run_moving_coded_board.py','run_moving_coded_board_stage.py','audit_moving_coded_board.py','summarize_moving_coded_board.py','inspect_missing_board_labels.py','write_moving_coded_board_report.py','verify_moving_coded_board.py','moving_coded_board_protocol.json','moving_coded_board_protocol_preflight_v1.json','coded_board_preflight_v1.py','test_coded_board_preflight_v1.txt','run_moving_coded_board_preflight_v1.py','audit_moving_coded_board_preflight_v1.py','moving_coded_board_status.json','moving_coded_board_startup_failure_status.json','moving_coded_board_startup_failure.log','moving_coded_board_run.log','moving_coded_board_geometry.log','moving_coded_board_summary.log','moving_coded_board_missing_labels.log','coded_board_red.log','coded_board_red_feature.log','coded_board_red_complete.log','coded_board_green.log','coded_board_front_red.log','coded_board_front_green.log','moving_coded_board_full_suite.log','moving_coded_board_full_suite_v2.log','moving_coded_board_verify_initial_failure.log']]
files += list((H/'figures').glob('moving_coded_board*.png'))+[W/'reports/benchmark_moving_coded_board_rgb_pose_20261004.md']
if (H/'moving_coded_board_review.json').exists():files.append(H/'moving_coded_board_review.json')
v=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=old_checks,stage_tests=111,whole_research_complete=False,actual_sequences=1,startup_failure_before_capture=True,geometry_gate_pass=False,matching_runs=0,
    actual_motion_nonraw_17_exact=True,raw_origin_delta_s=-.02,raw_index_delta=-1,history_shifts_once=120,physics_samples=601,captures=121,requests=120,video_frames_each=121,camera_max_age_s=max(ages),full_expected=420,recovered=356,correct_2px=356,source_and_artifact_sha256={str(f):sha(f) for f in files})
(H/'moving_coded_board_verification.json').write_text(json.dumps(v,indent=2)+'\n')
print('VERIFIED',v['verified_utc'],old_checks,'old SHA',len(files),'new identities; 356/420, geometry rejection preserved, matching0')
