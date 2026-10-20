"""Author verification; independent saved-array reconstruction, no simulator/predictor rerun."""
import json,hashlib,datetime
from pathlib import Path
import cv2,numpy as np
from observation_schedule import schedule
H=Path(__file__).resolve().parent;W=H.parent.parent;o=H/'matte_coded_board_run'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
p=json.loads((H/'matte_coded_board_protocol.json').read_text());old_checks=0
for f,s in {**p['source_sha256'],**p['previous_artifacts']}.items():assert sha(f)==s,f;old_checks+=1
status=json.loads((H/'matte_coded_board_status.json').read_text());assert status['exit_code']==0
pre=json.loads((H/'moving_coded_board_startup_failure_status.json').read_text());assert pre['exit_code']==-11
assert len(list((H/'moving_coded_board_startup_failure/frames').iterdir()))==0
identity=json.loads((o/'identity.json').read_text());assert (identity['actions'],identity['captures'],identity['saved_frames'],identity['reset_count'],identity['recorded_video_frames'])==(120,121,6,1,121)
for f,s in identity['source_sha256'].items():assert sha(f)==s,f
requests=np.load(o/'requests.npz')['requests'];np.testing.assert_equal(requests,schedule('forward'))
phy=np.load(o/'physics.npz');original=np.load(H/'moving_coded_board_run/physics.npz');assert len(phy['simulation_time_s'])==601
same=[k for k in phy.files if k not in ['raw_simulation_time_s','raw_physics_step_index']]
assert len(same)==17
for k in same:np.testing.assert_equal(phy[k],original[k])
np.testing.assert_allclose(phy['raw_simulation_time_s']-original['raw_simulation_time_s'],0.,atol=1e-8,rtol=0)
np.testing.assert_equal(phy['raw_physics_step_index']-original['raw_physics_step_index'],0)
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
    cap=cv2.VideoCapture(str(o/(name+'.mp4')));assert int(cap.get(cv2.CAP_PROP_FRAME_COUNT))==121 and cap.get(cv2.CAP_PROP_FPS)==10.
    decoded=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        decoded+=1;assert frame is not None
    assert decoded==121;cap.release()
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
assert (expected,recovered,correct)==(420,420,420) and assess['geometry_gate_pass']
summary=json.loads((o/'summary.json').read_text());stage_summary=json.loads((o/'stage_summary.json').read_text())
assert summary['missing_tags']==0 and summary['geometry_gate_pass'] and not summary['matching_run'] # Explicit retained legacy placeholder, superseded.
assert stage_summary['matching_runs']==6 and stage_summary['geometry_gate_pass'] and not stage_summary['matching_admission']
comparison=json.loads((o/'material_comparison.json').read_text());assert comparison['old_missing16_now_rgb_detected']==16 and comparison['old_missing16_now_truequad_decoded']==16
assert all(r['matte']['border_white_cells']==0 for r in comparison['paired_baseline_missing_labels'])
assert p['matching_budget']['max_anchors_per_frame']*p['matching_budget']['max_frames']==144>=p['matching_budget']['numeric_gate']['min_queries']
oldp=json.loads((H/'moving_coded_board_protocol_preflight_v1.json').read_text())
for source,snapshot in [('coded_board.py','coded_board_preflight_v1.py'),('test_coded_board.py','test_coded_board_preflight_v1.txt'),('run_moving_coded_board.py','run_moving_coded_board_preflight_v1.py'),('audit_moving_coded_board.py','audit_moving_coded_board_preflight_v1.py')]:assert sha(H/snapshot)==oldp['source_sha256'][str(H/source)]

# Independent V1 bookkeeping from all saved costs and ray labels, no new predictor run.
from board_material import select_rgb_anchors
manifest=json.loads((o/'v1_sparse/manifest.json').read_text());match=json.loads((o/'v1_sparse/assessment.json').read_text());assert manifest['saved_utc']==match['predictions_saved_utc']<match['labels_evaluated_utc']
assert manifest['geometry_not_loaded'] and set(manifest['sanitized_fields'])=={'frame','time_s','eye','rgb','pose','mask'}
qn=kept=good=0;errors=[]
for row in manifest['rows']:
    i=row['frame'];saved=np.load(row['path']);assert sha(row['path'])==row['sha256']
    detection=np.load(o/'rgb_detections'/f'frame_{i:03d}_left.npz');q,ids=select_rgb_anchors(detection['ids'],detection['corners'],masks['left'],24)
    np.testing.assert_equal(q,saved['anchors_yx']);np.testing.assert_equal(ids,saved['anchor_id_corner']);assert len(q)==24
    past=[f for f in p['selected_indices'] if f<i][-3:];expected_sources=[[i,'right',states[i]['time_s']]]+[[k,e,states[k]['time_s']] for k in past for e in ['left','right']]
    assert row['source_ids']==expected_sources and all(k<=i and (k!=i or e=='right') for k,e,t in row['source_ids'])
    costs=saved['source_cost'];count=np.isfinite(costs).sum(2);ordered=np.sort(costs,axis=2)
    lo=np.take_along_axis(ordered,np.maximum((count-1)//2,0)[:,:,None],2)[:,:,0];hi=np.take_along_axis(ordered,np.maximum(count//2,0)[:,:,None],2)[:,:,0]
    np.testing.assert_equal(saved['cost'],np.where(count>=1,(lo+hi)/2,np.inf).astype(np.float32))
    bits=np.zeros_like(saved['view_bits'])
    for j in range(len(row['source_ids'])):bits|=saved['source_valid'][:,:,j].astype(np.uint8)*np.uint8(1<<j)
    np.testing.assert_equal(bits,saved['view_bits'])
    depths=saved['depths_m'];np.testing.assert_allclose(depths,np.sort(1/np.linspace(1/3,1/.5,128)),atol=0,rtol=0)
    wi=saved['cost'].argmin(1);bc=saved['cost'][np.arange(len(q)),wi];finite=np.isfinite(bc)
    np.testing.assert_equal(saved['best_index'],wi);np.testing.assert_equal(saved['raw_depth_m'],np.where(finite,depths[wi],np.nan).astype(np.float32))
    celllo=depths.copy();cellhi=depths.copy();mid=(depths[:-1]+depths[1:])/2;celllo[1:]=mid;cellhi[:-1]=mid
    comp=np.isfinite(saved['cost'])&(saved['cost']<=bc[:,None]+.05)
    raynorm=np.sqrt(1+((q[:,1]-319.5)/584.8888888888889)**2+((q[:,0]-239.5)/584.8888888888889)**2)
    span=np.where(finite,(np.max(np.where(comp,cellhi,-np.inf),1)-np.min(np.where(comp,celllo,np.inf),1))*raynorm,np.inf)
    np.testing.assert_equal(saved['interval_ray_width_m'],span.astype(np.float32))
    cf=finite&(wi>0)&(wi<127)&(bc<=.2)&(span<=.1+1e-8);np.testing.assert_equal(cf,saved['confidence'])
    pose=np.array(states[i]['left_pose']);rays=np.c_[(q[:,1]-319.5)/584.8888888888889,(q[:,0]-239.5)/584.8888888888889,np.ones(len(q))];world=rays@(pose[:3,:3]@np.diag([1.,-1.,-1.])).T
    z=(geometry['plane_x']-pose[0,3])/world[:,0];labels=np.load(o/'v1_sparse'/f'labels_{i:03d}.npz');np.testing.assert_equal(z,labels['expected_depth_m']);assert labels['eligible_label'].all()
    error=np.abs(saved['raw_depth_m']-z)*raynorm;np.testing.assert_allclose(error,labels['error_m'],atol=1e-12,rtol=0,equal_nan=True)
    ok=cf&(error<=.1);np.testing.assert_equal(ok,labels['correct']);qn+=len(q);kept+=int(cf.sum());good+=int(ok.sum());errors.extend(error[cf])
assert (qn,kept,good)==(144,35,33) and not match['gate_pass']
np.testing.assert_allclose(np.percentile(errors,90),match['conditional_p90_m'],atol=1e-12,rtol=0)
material=json.loads((o/'material_identity.json').read_text());assert material['roughness']==1 and material['specularColor']==[0,0,0] and material['metallic']==0 and material['clearcoat']==0
files=[f for f in o.rglob('*') if f.is_file()]
files+=[H/f for f in ['board_material.py','test_board_material.py','run_matte_coded_board.py','run_matte_coded_board_stage.py','audit_matte_coded_board.py','summarize_matte_coded_board.py','inspect_matte_board_labels.py','assess_board_material.py','run_matte_board_matching.py','audit_matte_v1_failure.py','write_board_material_report.py','verify_board_material.py','matte_coded_board_protocol.json','matte_coded_board_status.json','board_material_red.log','board_material_red_features.log','board_material_green.log','board_material_full_suite.log','run_matte_coded_board.py.diff','audit_matte_coded_board.py.diff','matte_coded_board_run.log','matte_coded_board_geometry.log','matte_coded_board_summary.log','board_material_comparison.log','matte_board_matching.log','matte_v1_failure.log']]
files+=list((H/'figures').glob('matte*board*.png'))+[H/'figures/board_material_paired_missing_labels.png',W/'reports/benchmark_coded_board_material_and_sparse_depth_20261004.md']
if (H/'board_material_review.json').exists():files.append(H/'board_material_review.json')
v=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=old_checks,stage_tests=115,whole_research_complete=False,actual_sequences=1,geometry_gate_pass=True,matching_runs=6,matching_admission=False,matching_queries=qn,matching_retained=kept,matching_correct=good,matching_p90_m=float(np.percentile(errors,90)),actual_motion_nonraw_17_exact=True,raw_origin_delta_s=0,raw_index_delta=0,history_shifts_once=120,physics_samples=601,captures=121,requests=120,video_frames_each=121,camera_max_age_s=max(ages),full_expected=420,recovered=420,correct_2px=420,source_and_artifact_sha256={str(f):sha(f) for f in files})
(H/'board_material_verification.json').write_text(json.dumps(v,indent=2)+'\n')
print('VERIFIED',v['verified_utc'],old_checks,'old SHA',len(files),'new; geometry420/420 pass; V1 33/144 rejected')
