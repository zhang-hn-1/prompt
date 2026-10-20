"""Measured sustained speed vs fixed task duration; no universal speed bound."""
from pathlib import Path
import hashlib,json
import numpy as np
H=Path(__file__).resolve().parent;r=H/'online_bank_hold_navigation_run';p=np.load(r/'physics.npz');refpath=H/'online_provenance_evaluation_reference_e1.npz';ref=np.load(refpath)['position_world_xy'];speed=np.linalg.norm(p['linear_velocity_world_m_s'][:,:2],axis=1);time=p['simulation_time_s'];active=(time>=6-1e-5)&(time<28.4-1e-5);tail=(time>=15-1e-5)&(time<28.4-1e-5)
length=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum());straight=float(np.linalg.norm(ref[-1]-ref[0]))
row=dict(scope='offline feasibility/time-budget diagnostic, no cap change or performance certificate',frozen_reference_length_m=length,straight_start_goal_m=straight,scoring_budget_s=35.,existing_settle_s=6.,remaining_nominal_s=29.,required_mean_reference_speed_after_settle_m_s=length/29,required_mean_straight_goal_speed_after_settle_m_s=straight/29,observed_active_actual_speed_mean_m_s=float(speed[active].mean()),observed_active_actual_speed_median_m_s=float(np.median(speed[active])),observed_active_actual_speed_p90_m_s=float(np.percentile(speed[active],90)),observed_15_to28p4_speed_mean_m_s=float(speed[tail].mean()),no_universal_speed_bound_claim=True,
 interpretation='present main.3 unknown-risk policy has insufficient measured sustained progress for original35s reference. This measured mean is not a hard physical speed bound; another valid navigation route need not equal reference length, but even shortest straight goal mean is larger.',
 source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(__file__),r/'physics.npz',refpath)})
(H/'online_bank_hold_time_budget_diagnostic.json').write_text(json.dumps(row,indent=2)+'\n');print(json.dumps(row,indent=2))
