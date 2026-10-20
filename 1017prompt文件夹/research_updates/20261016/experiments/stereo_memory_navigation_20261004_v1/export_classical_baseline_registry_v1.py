"""Reproducible evidence index for completed TaskH baseline, not a new benchmark run."""
from pathlib import Path
import json,hashlib,math
import numpy as np
H=Path(__file__).resolve().parent
RUNS={
 'known_pose': ['sgbm_baseline35_e0_coverage_run_v1','sgbm_baseline35_e1_coverage_run_v1','sgbm_baseline35_e2_run_v1'],
 'estimated_pose':['native_pose_baseline35_e0_run_v1','native_pose_baseline35_e1_coverage_run_v1','native_pose_baseline35_e2_coverage_run_v1']}
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def summarize(name):
 d=H/name;states=json.loads((d/'states.json').read_text());contacts=json.loads((d/'physics_contacts.json').read_text());termination=json.loads((d/'termination.json').read_text());episode=json.loads((d/'task_episode.json').read_text());goal=np.asarray(episode['actual_episode']['goal_position_world_m'][:2]);available=[c for c in contacts if c['available']];hits=[c for c in available if c['contact']];contact_time=min((c['simulation_time_s'] for c in hits),default=None)
 distance=lambda s:float(np.linalg.norm(np.asarray(s['position'][:2])-goal))
 assert states and all(math.isfinite(distance(s)) for s in states)
 before=[s for s in states if contact_time is None or s['time_s']<contact_time]
 last=states[-1];pre=before[-1] if before else None
 reached=termination['termination']=='success' and contact_time is None and distance(last)<=.2
 assert not reached # current archived baseline results only; no new success inferred from cursor
 return dict(run=name,scope=episode['actual_task'],termination=termination['termination'],budget_s=termination['budget_s'],policy_request_intervals=termination['actions'],captures=len(states),first_contact_time_s=contact_time,available_contact_samples=len(available),unavailable_contact_samples=len(contacts)-len(available),post_contact_capture_possible=contact_time is not None and last['time_s']>=contact_time,terminal_capture_time_s=last['time_s'],terminal_capture_goal_distance_m=distance(last),last_pre_contact_capture_time_s=None if pre is None else pre['time_s'],last_pre_contact_goal_distance_m=None if pre is None else distance(pre),arrived=reached,not_strict_original_task=True,original_time_rule_preserved=True,videos={k:str(d/(k+'.mp4')) for k in ['stereo_view','third_person']},source_identity=str(d/'identity.json'),original_manifest=str(d/'artifact_manifest_post_close.json'),input_sha256={str(d/n):sha(d/n) for n in ['termination.json','task_episode.json','states.json','physics_contacts.json','identity.json','artifact_manifest_post_close.json']})
if __name__=='__main__':
 rows={layer:[summarize(n) for n in names] for layer,names in RUNS.items()}
 out=dict(schema='classical_stereo_memory_TaskH_baseline_evidence_registry_v1',task_assumptions=['fixed camera and light','flat ground prior','safe starting region prior','controlled unknown exploration','static environment assumption'],priors_are_not_observations=True,known_pose_is_privileged=True,estimated_pose_initial_gauge_and_low_level_ideal_IMU_privileged=True,not_pure_RGB_only=True,not_real_time_10Hz_claim=True,original_strict_cold_start_result=None,success_counts={layer:dict(arrived=sum(r['arrived'] for r in records),episodes=len(records)) for layer,records in rows.items()},layers=rows,new_physics_runs=0,new_training_runs=0,baseline_implementation_and_evaluation_delivered=True,robust_navigation_accepted=False,whole_research_mission_complete=False,exporter_sha256=sha(Path(__file__)))
 p=H/'classical_baseline_registry_v1.json';assert not p.exists(),'immutable registry already exported';p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(dict(success_counts=out['success_counts'],registry=str(p)),indent=2))
