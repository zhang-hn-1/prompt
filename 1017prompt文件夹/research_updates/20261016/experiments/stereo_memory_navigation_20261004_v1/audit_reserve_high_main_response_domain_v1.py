"""Finite measured command-domain audit, not a fitted motion model or prediction."""
from pathlib import Path
import json,hashlib,numpy as np
H=Path(__file__).resolve().parent
D=H/'local_reserve_fixed_reference_paced_run_v1'
seal=json.loads((D/'artifact_manifest_post_close.json').read_text())
for f,h in seal['artifact_sha256'].items():assert hashlib.sha256(Path(f).read_bytes()).hexdigest()==h,f
rows=json.loads((D/'policy_records.json').read_text());p=dict(np.load(D/'physics.npz'))
hi=[r for r in rows if r.get('local_reserve_pacing_applied')]
blocks=np.array([r['frame'] for r in hi]);mask=np.isin(p['action_block_index'],blocks)
def interval(a):return dict(min=float(np.min(a)),max=float(np.max(a)))
q=np.array([r['request'] for r in hi]);t=np.array([r['time_s'] for r in hi]);roll=np.array([r['straight_pacing_bank_rad'] for r in hi]);speed=np.linalg.norm(p['linear_velocity_world_m_s'][mask,:2],axis=1)
r=dict(sealed_items_verified=len(seal['artifact_sha256']),actual_high_main_request_rows=len(hi),first_last_request_time_s=interval(t),requested_aux_rad=interval(q[:,1::2]),requested_bank_target_deg=interval(q[:,1::2]*.75*180/np.pi),current_measured_bank_deg=interval(roll*180/np.pi),physical_roll_during_boost_deg=interval(p['roll_world_rad'][mask]*180/np.pi),actual_speed_during_boost_m_s=interval(speed),physical_samples=int(mask.sum()),steady_domain_demonstrated=False,reason='Contiguous1.6-second request burst followed by main.5, changing corrective aux and opposite-sign current bank. No held constant main1 moderate-bank steady sequence; a finite transient is not general speed-curvature admission.',old_high_main_aux0_point_not_generalized=True,old_four_point_model_frozen=True,no_fit=True,no_new_physics=True,next_question='Before any new main producer, determine whether existing archived high-main/nonzero-aux response contains a genuinely held command interval and actual stability; use actual command and roll/XY definitions, not old curvature labels or future applied inputs.',whole_mission_complete=False)
out=H/'local_reserve_high_main_response_domain_audit_v1.json';assert not out.exists();out.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
