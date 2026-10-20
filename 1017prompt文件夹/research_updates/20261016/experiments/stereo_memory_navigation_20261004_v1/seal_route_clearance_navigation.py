"""Seal the sole route-consumer candidate and unchanged numerical gates."""
import json,hashlib
from pathlib import Path
from datetime import datetime,timezone
from route_clearance_consumer import STOP_RADIUS_M
H=Path(__file__).resolve().parent
read=lambda p:json.loads(p.read_text());sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
dest=H/'route_clearance_navigation_protocol.json'
assert not dest.exists(), 'keep the first protocol; no resealing after results'
old=read(H/'online_bank_hold_navigation_run/experiment_identity.json')
old_verification=read(H/'online_route_execution_verification.json')
for s,v in old_verification['source_and_artifact_sha256'].items():assert sha(Path(s))==v,s
files=[H/n for n in ('route_clearance_consumer.py','route_clearance_policy.py','test_route_clearance_consumer.py',
    'run_route_clearance_navigation.py','run_route_clearance_navigation_phase.py','route_geometry_audit.py',
    'online_bank_hold_navigation_run/physics.npz','online_bank_hold_navigation_run/vision_records.json',
    'online_bank_hold_navigation_run/policy_states.json','online_bank_hold_navigation_run/execution_states.json',
    'online_bank_hold_navigation_run/episode_golden_e1_0001/metrics.json','online_provenance_evaluation_reference_e1.npz')]
files += [Path(s) for s in old['source_sha256']]
files += [H.parents[1]/'docs/superpowers/plans/2026-10-04-route-stop-clearance-single-consumer.md']
protocol=dict(frozen_at_utc=datetime.now(timezone.utc).isoformat(),candidate_count=1,max_actual_new_runs=1,
    budget_s=35.,baseline='archived online_bank_hold_navigation_run; no rerun',episode='golden_e1_0001',
    sole_change='event-latched planner point-clearance .6 to EXISTING stop-disc radius; no other control changes',
    trigger='ordinary planning call: old route first1m with actual-centre bridge has observed hazard clearance below existing stop radius, while current actual centre is outside disc',
    stop_radius_m=STOP_RADIUS_M,certified_stopping_bound=False,trigger_uses_future_truth=False,
    unchanged=['camera rig/calibration/masks/lighting','map/provenance/unknown semantics','PID/smoother/protocol/supervisor',
        'main .3 cap','pointgoal/geometric 1m target','bank hold','direction latch/five-low/rebound','stopping consumer and radius'],
    prefix_gate='same actual physical/currentpacket/policy/PID/smoother/latch/action prefix through planning event; raw origin disclosed; first changed request separately identified',
    local_gate=dict(window_s=6.,no_contact=True,path_length_nonregression=True,original_goal_progress_nonregression=True,
        fewer_veto_blocks=True,meaningful_extra_goal_or_event_reference_progress_m=.05,
        evaluation_reference='same event-time B0 path, only offline; not candidate index; no alternative-plan future labels'),
    whole_gate=dict(reference_progress_nonregression=True,lateral_rms_nonregression=True,lateral_peak_nonregression=True,goal_distance_nonregression=True,no_contact=True),
    navigation_gate='actually reach original goal without actual contact in original35s; no new RGB/formal success-rate claim',
    tolerances=dict(space_m=1e-5,time_s=1e-4),old_RGB_rejection_unchanged=True,
    source_sha256={str(f):sha(f) for f in files})
dest.write_text(json.dumps(protocol,indent=2)+'\n');print(protocol['frozen_at_utc'])
