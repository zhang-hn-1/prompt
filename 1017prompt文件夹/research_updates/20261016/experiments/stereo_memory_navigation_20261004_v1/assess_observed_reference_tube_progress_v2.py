"""Frozen physical progress and navigation layers, no path-index success."""
from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent;D=H/'observed_reference_tube_navigation_run_v1'
assert json.loads((H/'observed_reference_tube_navigation_status_v1.json').read_text())['status']=='finished'
runtime=json.loads((H/'observed_reference_tube_sparse_memory_navigation_runtime_verification_v1.json').read_text());boundary=json.loads((H/'observed_reference_tube_boundary_verification_v1.json').read_text());source=json.loads((H/'observed_reference_tube_sparse_memory_navigation_assessment_v1/assessment.json').read_text());maps=json.loads((H/'observed_reference_tube_memory_consumption_assessment_v1.json').read_text());membership=json.loads((H/'observed_reference_tube_sparse_membership_v1/assessment.json').read_text())
z=np.load(D/'physics.npz');xy=z['position_world_xy'].astype(np.float64);t=z['simulation_time_s'];goal=np.array([3.300000005960465,5.960465e-9]);dist=np.linalg.norm(xy-goal,axis=1);axis=xy[:,0]-(-3.2999999940395353);term=json.loads((D/'termination.json').read_text());N=len(np.load(D/'requests.npz')['requests']);assert len(t)==5*N+1
contact=runtime['physical_contacts'];at546=dict(scope='NOT_REACHED',passed=False)
if N>=546:
 k=2730;at546=dict(scope='MEASURED_ORIGINAL54_6',goal_remaining_m=float(dist[k]),axis_progress_m=float(axis[k]),passed=bool(dist[k]<=4.1829005 and axis[k]>=2.427377623 and contact==0));saved=json.loads((D/'local54_6_gate.json').read_text());assert abs(saved['goal_remaining_m']-dist[k])<1e-10 and abs(saved['axis_progress_m']-axis[k])<1e-10
physical_goal=term['termination']=='success' and dist[-1]<=.2 and contact==0
sourcepass=all(source['checks'].values()) and all(source['prefix54_6_source_checks'].values()) and membership['semantic_subset95'] and membership['prefix54_6_semantic_subset95']
causal=json.loads((H/'observed_reference_tube_policy_causal_assessment_v2.json').read_text());shadow=json.loads((H/'observed_reference_tube_online_shadow_verification_v1/assessment.json').read_text())
interfacepass=boundary['handoff_frame'] is not None and runtime['execution_ticks_exact']==5*N and maps['maps_exact']==N and causal['requests_exact']==N and shadow['actual_shadow_rows_and_candidate_requests_exact'] and shadow['shadow_on_off_candidate_requests_references_exact']
result=dict(decision='PASS_ONE_CONDITIONAL_KNOWNPOSE_TASKH_NAVIGATION' if physical_goal and sourcepass and interfacepass else 'REJECT_UNIQUE_OBSERVED_REFERENCE_TUBE_NAVIGATION',physical_goal=bool(physical_goal),source_subset_pass=bool(sourcepass),interface_pass=bool(interfacepass),gate54_6=at546,termination=term,final_goal_remaining_m=float(dist[-1]),axis_progress_final_m=float(axis[-1]),path_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),physical_contact=contact,handoff_time_s=boundary['handoff_time_s'],not_full_causal_AB=True,not_source_complete_3D=True,known_GT_pose=True,estimated_pose_actor_NOTRUN=True,formal_original35_NOTRUN=True,world_model_increment_not_newly_passed=True,whole_research_complete=False)
(H/'observed_reference_tube_progress_assessment_v2.json').write_text(json.dumps(result,indent=2)+'\n')
fig,axs=plt.subplots(1,2,figsize=(11,4));axs[0].plot(xy[:,0],xy[:,1],label='actual XY');axs[0].scatter(*xy[0],marker='o',label='start');axs[0].scatter(*goal,marker='*',label='goal');axs[0].set_aspect('equal');axs[0].legend();axs[0].set_xlabel('world X [m]');axs[0].set_ylabel('world Y [m]');axs[1].plot(t,dist,label='true goal distance');axs[1].plot(t,axis,label='true axis progress');axs[1].axvline(boundary['handoff_time_s'],ls='--',label='measured handoff');axs[1].axvline(54.6,color='gray',ls=':');axs[1].legend();axs[1].set_xlabel('simulation time [s]');fig.tight_layout();fig.savefig(H/'observed_reference_tube_trajectory_v1.png',dpi=150);plt.close(fig);print(json.dumps(result,indent=2))
