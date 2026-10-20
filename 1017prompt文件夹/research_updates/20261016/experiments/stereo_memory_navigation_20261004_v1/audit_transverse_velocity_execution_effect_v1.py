"""Postclosed actual effect, not a model, counterfactual or acceptance gate."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 ds={a:H/f'transverse_velocity_feedback_e1_{a}_run_v1' for a in ('b0','candidate')}
 pp={a:np.load(d/'physics.npz') for a,d in ds.items()};rr={a:json.loads((d/'policy_records.json').read_text()) for a,d in ds.items()}
 result={};inputs={}
 for key in ('smoothed_target','applied_action','joint_position_rad','joint_velocity_rad_s','position_world_xy','linear_velocity_world_m_s'):
  a,b=pp['b0'][key],pp['candidate'][key];assert a.shape==b.shape
  diff=np.any(a!=b,axis=1);ids=np.flatnonzero(diff)
  result[key]=dict(first_different_sample=None if not len(ids) else int(ids[0]),first_different_time_s=None if not len(ids) else float(pp['candidate']['simulation_time_s'][ids[0]]),different_samples=int(diff.sum()),maximum_abs_difference=float(np.max(abs(a-b))),paired_future_labels=False)
 arms={}
 for name,d in ds.items():
  rows=rr[name];normal=[r for r in rows if r.get('transverse_feedback_eligible',False)]
  velocity=np.asarray([r['transverse_feedback_velocity_m_s'] for r in normal]);correction=np.asarray([r['transverse_feedback_correction_rad'] for r in normal]);f=next((d/'feedback').glob('*.json'));feedback=json.loads(f.read_text());assert len(feedback)==1750
  arms[name]=dict(normal_forward_eligible_count=len(normal),reference_normal_velocity_rms_m_s=float(np.sqrt(np.mean(velocity**2))),reference_normal_velocity_abs_p90_m_s=float(np.percentile(abs(velocity),90)),aux_correction_abs_p90_rad=float(np.percentile(abs(correction),90)),PID_clipped_ticks=sum(x['pid_clipped'] for x in feedback),downstream_blocked_ticks=sum(x['downstream_blocked'] for x in feedback),supervisor_active_ticks=sum(x['supervisor_active'] for x in feedback),feedback_tick_count=len(feedback),actual_override_count=sum(r.get('transverse_feedback_override',False) for r in rows))
  for file in (d/'physics.npz',d/'policy_records.json',f,d/'artifact_manifest_post_close.json'):inputs[str(file)]=sha(file)
 out=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),actual_pair_effect=result,arms=arms,input_sha256=inputs,source_sha256={str(Path(__file__).resolve()):sha(Path(__file__))},notes=['Any postfork difference is actual complete closed-loop response, not isolated same-state impulse response.','Per-arm reference normals and eligible durations may differ; these are descriptive, not a new cross-error gate.','No future execution value was an online consumer input; this is postclosed evidence only.','PID clipping/supervisor activation counts do not prove an implementation fault.'],no_new_solver_physics_fit=True,whole_mission_complete=False)
 p=H/'transverse_velocity_execution_effect_v1.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
if __name__=='__main__':main()
