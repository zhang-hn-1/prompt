"""Pre-run one-candidate numerical seal. Not imported by assessment."""
import datetime,hashlib,json
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent
root=H/'short_branch_runs/development'
p=np.load(root/'physics.npz');j=375;speed=np.linalg.norm(p['linear_velocity_world_m_s'][:, :2],axis=1)
a=json.loads((H/'short_branch_assessment.json').read_text())['development']
files=[H/'run_stop_bank_hold.py',H/'stop_bank_hold.py',H/'test_stop_bank_hold.py',H/'run_short_branch.py',H/'short_branch.py',H/'execution_snapshot.py',H/'spatial_memory.py',H/'capture_transaction.py',H/'render_flush.py',root/'physics.npz',root/'requests.npz',root/'execution_states.json',root/'identity.json',H/'short_branch_assessment.json']
files.extend(sorted(root.glob('packets/*.npz')))
protocol=dict(frozen_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),candidate_count=1,new_sequence_count=1,baseline='archived short_branch_runs/development; no rerun',seconds=13.5,branch_time_s=7.5,
  sequence='6s rawzero +1.5s main+.3/encodedaux+30;6s main0/encodedaux+30 derived from last known request',
  intervention='only raw bank reference during main stop; keep earlier request rather than set0; no measured/applied angle used as held target',
  fixed=['rig','PID gains .4/.1/.7','protocol','smoother','supervisor','physics','initial state/seed','capture transaction','source memory','main commands','scoring horizon'],
  prefix_gate='all17nonraw physical fields, relative clock, all75 requests and complete execution snapshots through7.5s exact; raw origins reported separately',
  brake_gate=dict(actual_qualification_rule='five consecutive10Hz XYspeed<.05; reset on rebound/nonfinite',
    qualification_delay_max_s=a['first_stop_qualification']['delay_from_zero_s'],
    windows=a['stop_windows'],speed_peak_max_m_s=a['speed_peak_after_zero_until_6s_m_s'],end_one_second_mean_speed_max_m_s=float(speed[-51:].mean()),
    no_post_qualification_rebound=True,spatial_numeric_tolerance_m=1e-5,speed_numeric_tolerance_m_s=1e-5,time_numeric_tolerance_s=1e-4,
    meaningful_improvement='at least10% lower6s radialpeak OR at least.2s earlier firstqualification; all nonregression clauses also must pass'),
  progress_evaluation='fixed world localgoal from6s currentyaw(.8,-.3); report alonggoal progress/side-error separately, never reward braking drift as navigation success',
  exclusion='openplane no hazard means contacts are not obstacle safety evidence; depth+truepose diagnostic, no RGB/admitted primitive/formal35s results',
  stop_after='one run; rejection causes diagnosis not rerun/threshold scan/network/table expansion',
  source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files})
path=H/'stop_bank_hold_protocol.json'
assert not path.exists() and not (H/'stop_bank_hold_run').exists()
path.write_text(json.dumps(protocol,indent=2)+'\n')
print(json.dumps({k:v for k,v in protocol.items() if k!='source_sha256'},indent=2))
