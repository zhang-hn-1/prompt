"""Post-close measured outcomes, not source admission or causal RGB A/B."""
from pathlib import Path
import json, hashlib, argparse
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from baseline35_metrics_v1 import assess, observation_gap_diagnostics
from matched_progress_v1 import matched_progress

H = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--condition', choices=['e0'], required=True)
name = parser.parse_args().condition
assert name in ('e0','e1')
D = H / 'native_pose_baseline35_e0_run_v1'
status = json.loads((H / 'native_pose_baseline35_e0_status_v1.json').read_text())
assert status['status']=='finished' and status['normal_close_confirmed']
manifest = json.loads((D / 'artifact_manifest_post_close.json').read_text())
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p,h in manifest['artifact_sha256'].items())
s = json.loads((D/'states.json').read_text())
c = json.loads((D/'physics_contacts.json').read_text())
ep = json.loads((D/'task_episode.json').read_text())['actual_episode']
req = np.load(D/'requests.npz')['requests']
ph = np.load(D/'physics.npz')
rows = json.loads((D/'policy_records.json').read_text())
assert len(req) <= 350 and len(s) == len(req)+1 and len(rows) == len(req)
assert len(ph['simulation_time_s']) == len(req)*5+1
r = assess(s, [dict(time_s=a['simulation_time_s'], collision=a['contact'], available=a['available']) for a in c], ep['start_position_world_m'][:2], ep['goal_position_world_m'][:2])
B = H/'sgbm_baseline35_e0_coverage_run_v1'
base_manifest = json.loads((B/'artifact_manifest_post_close.json').read_text())
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p,h in base_manifest['artifact_sha256'].items())
bs = json.loads((B/'states.json').read_text())
base = json.loads((B/'sgbm_composition_assessment_v1.json').read_text())
cut = min(base['scored_end_s'], r['scored_end_s'])
start = np.asarray(ep['start_position_world_m'][:2]); goal = np.asarray(ep['goal_position_world_m'][:2])
unit = (goal-start)/np.linalg.norm(goal-start)
sample = max([a for a in s if a['time_s'] <= cut+1e-5], key=lambda a:a['time_s'])
matched=matched_progress(s,bs,cut,start,unit)
hand = [a for a in rows if a['front_view_acquisition'].get('standby_handoff')]
tracking = [a for a in rows if a['planner_act_called'] and any(abs(x)>1e-8 for x in a['request'])]
lat = np.asarray([a['end_to_end_wall_s'] for a in rows])
videos = {}
for n in ('stereo_view','third_person'):
    cap=cv2.VideoCapture(str(D/(n+'.mp4'))); fps=cap.get(cv2.CAP_PROP_FPS); count=0
    while True:
        ok,img=cap.read()
        if not ok: break
        if count in (0,len(s)//2,len(s)-1): cv2.imwrite(str(D/f'{n}_frame_{count:03d}.png'),img)
        count+=1
    cap.release()
    assert count == len(s) and abs(fps-10)<1e-6
    videos[n]=dict(decoded_frames=count,fps=fps)
r.update(scope='FIXED_CLASSICAL_BASELINE_NATIVE_IMU_ENCODER_POSE_TASK_PRIOR_NOT_PURE_RGB', condition=name, episode_id=ep['episode_id'], requests=len(req), captures=len(s), physics_samples=len(ph['simulation_time_s']), videos=videos, handoff_time_s=hand[0]['time_s'] if hand else None, tracking_request_count=len(tracking), base_outcome=base['outcome'], base_is_historical_reassessment=False, base_scored_end_s=base['scored_end_s'], matched_time_s=cut, matched_time_axis_progress_gain_m=matched['axis_gain_m'],candidate_matched_sample_time_s=matched['candidate_sample_time_s'],base_matched_sample_time_s=matched['base_sample_time_s'], terminal_axis_progress_gain_m=r['along_start_goal_axis_m']-base['along_start_goal_axis_m'], fullcausal_AB=False, source_accuracy_not_admission=True, navigation_success_requires_actual_goal_and_contact=True, old_rejections_preserved=True, observation_gaps=observation_gap_diagnostics(json.loads((D/'source_rows.json').read_text()),rows), cycle_wall_s=dict(median=float(np.median(lat)),p90=float(np.quantile(lat,.9)),maximum=float(max(lat)),fraction_over_0p1=float(np.mean(lat>.1))))
# An independent start-goal axis is not directed progress along E1's detour.
r['axis_is_not_along_detour_reference']=True
xy=np.asarray([a['position'][:2] for a in s]); scored_bs=[a for a in bs if a['time_s']<=35+1e-5]; bxy=np.asarray([a['position'][:2] for a in scored_bs])
fig,ax=plt.subplots(1,2,figsize=(10,4))
ax[0].plot(bxy[:,0],bxy[:,1],label='SGBM known pose')
ax[0].plot(xy[:,0],xy[:,1],label='SGBM native IMU/encoder');ax[0].scatter(*goal,c='red');ax[0].legend();ax[0].set_aspect('equal')
ax[1].plot([a['time_s'] for a in scored_bs],np.linalg.norm(bxy-goal,axis=1),label='B0')
ax[1].plot([a['time_s'] for a in s],np.linalg.norm(xy-goal,axis=1),label='candidate');ax[1].legend();ax[1].set_xlabel('simulation seconds');ax[1].set_ylabel('goal distance m')
fig.tight_layout();fig.savefig(D/'native_pose_trajectory_and_goal_v2.png',dpi=150);plt.close(fig)
breq=np.load(B/'requests.npz')['requests'];length=min(len(req),len(breq));forks=np.flatnonzero(np.any(req[:length]!=breq[:length],axis=1));first=int(forks[0]) if len(forks) else None
bp=np.load(B/'physics.npz');prefix=(first if first is not None else length)*5+1
r.update(actual_matcher='frozen_SGBMStereo',base_matcher='frozen_SGBMStereo',comparison_base_historical=True,source_methods_differ_only_not_controller=False,only_pose_source_differs=True,first_common_request_difference_frame=first,common_request_blocks=length,nonraw_physics_before_request_fork_exact={k:np.array_equal(ph[k][:prefix],bp[k][:prefix]) for k in ph.files if not k.startswith('raw_') and k not in ('action_block_index','physics_step_in_block')},same_stream_Fast_source_shadow_executed=False)
native=json.loads((D/'native_predictions.json').read_text());lookup={round(float(a['time_s']),6):a for a in native if a['available']};errors=[]
for state in s:
    estimate=lookup[round(float(state['time_s']),6)];errors.append(float(np.linalg.norm(np.asarray(estimate['position_world'])[:2]-np.asarray(state['position'])[:2])))
assert all(a['native_pose_used'] and not a['known_GT_pose'] for a in rows)
assert all(not a['current_GT_pose_registration'] for a in json.loads((D/'source_rows.json').read_text()))
r.update(native_xy_final_error_m=errors[-1],native_xy_p90_error_m=float(np.percentile(errors,90)),initial_gauge_privileged=True,flat_height_prior=True,lowlevel_ideal_IMU_privileged=True,pure_RGB_admission=False,estimated_pose_used_online_at_registration_and_navigation=True)
(D/'native_pose_assessment_v2.json').write_text(json.dumps(r,indent=2)+'\n')
print({k:r[k] for k in ['condition','outcome','goal_remaining_m','along_start_goal_axis_m','handoff_time_s','tracking_request_count','matched_time_axis_progress_gain_m']})
