from pathlib import Path
import ast
H=Path(__file__).resolve().parent;s=(H/'assess_sgbm_baseline35_remaining_closed_v1.py').read_text()
s=s.replace("choices=['e0', 'e1']","choices=['e0']").replace("D = H / f'sgbbm_baseline35_{name}_coverage_run_v1'","D = H / 'native_pose_baseline35_e0_run_v1'")
s=s.replace("D = H / f'sgbm_baseline35_{name}_coverage_run_v1'","D = H / 'native_pose_baseline35_e0_run_v1'").replace("'sgbm_baseline35_remaining_status_v1.json'","'native_pose_baseline35_e0_status_v1.json'").replace("assert any(a['condition']==name and a['normal_close_confirmed'] for a in status['sequences'])","assert status['status']=='finished' and status['normal_close_confirmed']")
s=s.replace("B = H/('standby_handoff_e0_run_v1' if name=='e0' else 'standby_handoff_e1_transfer_run_v1')","B = H/'sgbm_baseline35_e0_coverage_run_v1'")
s=s.replace("(B/('standby_handoff_assessment_v1.json' if name=='e0' else 'standby_handoff_transfer_assessment_v2.json'))","(B/'sgbm_composition_assessment_v1.json')")
s=s.replace('FIXED_CLASSICAL_BASELINE_REMAINING_COVERAGE_KNOWNPOSE_TASK_PRIOR_NOT_STRICT_VISUAL','FIXED_CLASSICAL_BASELINE_NATIVE_IMU_ENCODER_POSE_TASK_PRIOR_NOT_PURE_RGB')
s=s.replace("label='B0 FastStereo'","label='SGBM known pose'").replace("label='fixed SGBM'","label='SGBM native IMU/encoder'")
s=s.replace("'sgbm_composition_trajectory_and_goal_v1.png'","'native_pose_trajectory_and_goal_v1.png'").replace("base_matcher='frozen_FastStereo'","base_matcher='frozen_SGBMStereo'").replace('source_methods_differ_only_not_controller=True','source_methods_differ_only_not_controller=False,only_pose_source_differs=True')
s=s.replace("(D/'sgbm_composition_assessment_v1.json')","(D/'native_pose_assessment_v1.json')")
needle="(D/'native_pose_assessment_v1.json').write_text"
addition="""native=json.loads((D/'native_predictions.json').read_text());lookup={round(float(a['time_s']),6):a for a in native if a['available']};errors=[]
for state in s:
    estimate=lookup[round(float(state['time_s']),6)];errors.append(float(np.linalg.norm(np.asarray(estimate['position_world'])[:2]-np.asarray(state['position'])[:2])))
assert all(a['native_pose_used'] and not a['known_GT_pose'] for a in rows)
assert all(not a['current_GT_pose_registration'] for a in json.loads((D/'source_rows.json').read_text()))
r.update(native_xy_final_error_m=errors[-1],native_xy_p90_error_m=float(np.percentile(errors,90)),initial_gauge_privileged=True,flat_height_prior=True,lowlevel_ideal_IMU_privileged=True,pure_RGB_admission=False,estimated_pose_used_online_at_registration_and_navigation=True)
"""
assert needle in s;s=s.replace(needle,addition+needle);ast.parse(s);(H/'assess_native_pose_baseline35_e0_closed_v1.py').write_text(s)
print('postclose assessor prepared')
