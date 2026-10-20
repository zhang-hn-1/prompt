from pathlib import Path
import json,hashlib,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;W=H.parents[1];out={}
fig,axs=plt.subplots(1,2,figsize=(11,4.3))
for arm in ['current','history']:
 o=H/f'short_obstacle_{arm}_run';a=json.loads((H/f'short_obstacle_{arm}_assessment.json').read_text());r=json.loads((o/'policy_records.json').read_text());s=json.loads((o/'states.json').read_text());xy=np.array([x['position'][:2] for x in s]);t=np.array([x['time_s'] for x in s]);A=np.array([x['local']['request'] for x in r]);ph=np.load(o/'physics.npz');zero=~np.any(A!=0,axis=1)
 out[arm]=dict(assessment=a,all_ten_raw_request_zero_blocks=int(zero.sum()),all_raw_zero=bool(zero.all()),maximum_goal_progress_m=float((xy[:,0]-xy[0,0]).max()),source_RGB_reset_confounded=True)
 axs[0].plot(xy[:,0],xy[:,1],label=arm);axs[1].plot(t,xy[:,0]-xy[0,0],label=arm)
axs[0].axvspan(-.35,.35,alpha=.15,color='red',label='divider x extent (offline only)');axs[0].scatter([-1.3,1.3],[0,0],c=['black','green']);axs[0].set(xlabel='world X (m)',ylabel='world Y (m)',title='Actual root trajectory; no path-index score');axs[0].legend();axs[1].set(xlabel='simulation time (s)',ylabel='true goal-direction progress (m)');axs[1].axhline(1.3,color='gray',ls='--',label='midplane');axs[1].legend();fig.tight_layout();fig.savefig(H/'short_obstacle_actual.png',dpi=150);plt.close(fig)
(H/'short_obstacle_navigation_comparison.json').write_text(json.dumps(out,indent=2)+'\n')
c=out['current']['assessment'];a=out['history']['assessment'];v=json.loads((H/'short_obstacle_visibility_diagnostic.json').read_text())
report=f'''# Short obstacle navigation: actual two-arm results (2026-10-07)

## Scope and controller identity

New 2.6m known-pose E1-scene diagnostic, declared symmetric endpoints X=-1.3 and +1.3 before execution. This is not original 6.6m E1 or a formal E0/E1/E2 result. Same frozen PriorRGBPolicy, CAD bilateral masking, Fast stereo eager eight iterations, ground/safe-start priors and explicitly permitted unknown exploration, roll PID, main request cap .3 rad/s, auxiliary limit .3 rad/s, camera rig and lighting. Current-only versus 10s history is the sole primary-consumer difference. Failed nominal/terminal candidates are not installed. Inherited original oracle-length/classification metadata is archival; SPL is not used. No known global route or renderer depth enters policy. True pose remains privileged, and depth source admission remains rejected.

## Actual results and acceptance

| primary | termination | simulation seconds | final distance m | true progress m | contact samples | calls/captures/physics/sources |
|---|---|---:|---:|---:|---:|---|
| current | {c['termination']} | {c['elapsed_s']} | {c['final_distance_m']:.9f} | {c['actual_goal_direction_progress_m']:.9f} | {c['contact_samples']} | {c['policy_calls']}/{c['captures']}/{c['physics_samples']}/{c['source_inferences']} |
| history | {a['termination']} | {a['elapsed_s']} | {a['final_distance_m']:.9f} | {a['actual_goal_direction_progress_m']:.9f} | {a['contact_samples']} | {a['policy_calls']}/{a['captures']}/{a['physics_samples']}/{a['source_inferences']} |

Neither arm crosses the actual X=0 midplane or reaches the goal under unchanged 35s/.2m/no-contact criteria. Both task gates FAIL. History has all {out['history']['all_ten_raw_request_zero_blocks']} ten-component raw request blocks zero; its .544m displacement is actual passive/closed-loop physical drift, not commanded forward route completion. Its smaller cross-track error cannot certify improvement when progress is absent. Current has {out['current']['all_ten_raw_request_zero_blocks']} all-zero blocks. No reliable memory navigation benefit is accepted. Independent simulation resets produce different RGB predictions even at matching poses; this is a task comparison, not isolated causal attribution to memory. Opposite-memory same-live shadow requests were never executed or assigned counterfactual trajectory labels.

## Observation failure, before any new fix

A separately sealed nine-exposure retrospective audit uses only completed logs: each short arm at frames 0/60/110, and older far-start yaw20 control at the same frames. Divider surface is a geometric proxy from the frozen USD cylinder definitions (radius .35m, height1m), not a renderer instance ID. GT endpoints identify that proxy only offline; no truth enters runtime or map reconstruction. The source/mask packets were finalized and saved before truth retrieval. The audit does not alter old gates.

In all six short-arm samples, roughly 160k left-visible unmasked divider pixels exist, but **zero project inside the right native image**. Thus the short cold start has no measured binocular correspondence on those visible divider surfaces; history cannot create unseen valid depth. At current frame60 there are two accepted source queries on the divider proxy, but neither is within .10m of its true endpoint. Per-arm source reconstruction also reports zero .10m-correct candidate queries over the full runs; this is a failed-source diagnostic, not certification of ghost obstacles. At history6s the consumer holds 58 historical hazard cells and reports observed_obstacle_blocks_route while raw requests stay zero; absence of collision does not validate those hazards.

The older far-start samples have right-depth-consistent, bilaterally unmasked divider pixels 31830/23252/21987, and packet divider queries accurate within .10m 1599/2914/3910 respectively. This is bounded historical evidence that observable approach coverage differs from near cold-start coverage, not a fresh network-validation pass. Right depth agreement tolerance .02m is a diagnostic classification only. The whole-run older depth gate remains rejected.

The .6m baseline and native vertical epipolar geometry matter. fB/416=.843589744m is the nominal cost-volume search scale; **max_disp is not a hard final disparity clip**, since recurrent updates add residual disparity. The necessary full-native overlap scale fB/479=.732637439m alone cannot guarantee object correspondence because pose, image bounds and occlusion also matter. At short frame0, divider axial depth median .80351m and disparity median436.75px; actual point projection, not just that formula, establishes zero in-image right correspondence for the sampled visible surfaces. Do not claim increasing search range alone fixes physical field-of-view loss.

## Actual verification evidence

Author independently ran both completed whole-arm assessors: current {c['identity_checks']} / history {a['identity_checks']} exact source-and-closed-artifact SHA checks; paired source/mask fields {c['exact_packet_mask_fields']}/{a['exact_packet_mask_fields']}; primary maps {c['exact_primary_map_arrays']}/{a['exact_primary_map_arrays']}; opposite-memory shadow diagnostic fields {c['exact_slow_history_shadow_fields']}/{a['exact_slow_history_shadow_fields']}. Each checks current/past source time, future-payload rejection, actual raw command blocks versus physics, unchanged endpoint derivation and gates, source/model identity, and all video frames. Four videos contain exactly121/351 frames per arm at10Hz, no scoring-tail frames. This is not a physical shadow on/off determinism experiment. Prelaunch full CPU suite was410 passing; scientific result is nevertheless rejected. Geometry audit is author-only unless a subsequent review explicitly covers it.

Files: short_obstacle_navigation_protocol.json, short_obstacle_navigation_status.json, per-arm assessment JSON/logs, two original run directories including physics/contact traces, RGB/depth namespace isolation, query/partner provenance, maps, smoother/execution states and videos. Geometry: short_obstacle_visibility_preanalysis.json / short_obstacle_visibility_diagnostic.json / audit_short_obstacle_visibility.py. Actual comparison and trajectory: short_obstacle_navigation_comparison.json / short_obstacle_actual.png.

## Next admissible work

Keep camera and lighting fixed, retain every original source/Oracle/IMU/M2/terminal refusal. First measure observability loss and actual retained hazard-source ages on the already completed far-start approach; distinguish never observed, invalid depth, expiring valid obstacle evidence and route/physical execution failure. This identifies whether one bounded active-observation maneuver or one provenance-preserving static-memory consumer is justified. No new threshold sweep, planner rewrite, network, formal benchmark or estimated-pose obstacle success is claimed. Learned memory/world-model increment and formal original35s E0/E1/E2 remain unfinished. Whole research is not complete.
'''
(W/'reports/benchmark_short_obstacle_navigation_20261007.md').write_text(report)
paths=[H/'short_obstacle_navigation_comparison.json',H/'short_obstacle_actual.png',W/'reports/benchmark_short_obstacle_navigation_20261007.md',H/'short_obstacle_visibility_preanalysis.json',H/'short_obstacle_visibility_diagnostic.json',H/'audit_short_obstacle_visibility.py',H/'short_obstacle_navigation_status.json',H/'short_obstacle_current_assessment.json',H/'short_obstacle_history_assessment.json']
closure=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='finished_two_actual_arms_both_rejected',review='author only pending one first-pass',whole_research_complete=False,owned_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
(H/'short_obstacle_author_closure.json').write_text(json.dumps(closure,indent=2)+'\n')
print('SHORT_TWO_ARM_REPORT_SAVED',c['short_obstacle_task_pass'],a['short_obstacle_task_pass'])
