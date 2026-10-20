"""Reconstruct execution from complete PAST feedback; archive indivisible responses."""
import json,hashlib
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from execution_snapshot import ExecutionClone,export_execution
from scripts.action_smoothing import ActionSmoother
from training.action_protocol import load_action_protocol
from roll_pid import BankPID,bank_from_quaternion
from guarded_response import physical_state,make_entry
H=Path(__file__).resolve().parent
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
p=load_action_protocol(P/'configs/data_contract.yaml');entries=[];evidence=[]
for name in ('positive','negative'):
    r=H/'large_bank_runs'/name;physics=np.load(r/'physics.npz');feedback=json.loads(next((r/'feedback').glob('*.json')).read_text())
    start=export_execution(SimpleNamespace(inner=ActionSmoother(.02),pid=BankPID(.4,.1,.7),blocked=False),p)
    clone=ExecutionClone.from_snapshot(start);errors=[];bankerrors=[];initial=None
    for tick,f in enumerate(feedback):
        bank=bank_from_quaternion(physics['root_quaternion_world_wxyz'][tick]);bankerrors.append(abs(bank-f['bank_rad']))
        a,s=clone.step(physics['raw_command_target'][tick+1],bank_rad=bank,aux_position_rad=physics['joint_position_rad'][tick,1],aux_velocity_rad_s=physics['joint_velocity_rad_s'][tick,1])
        errors.append([float(np.max(abs(a-physics['applied_action'][tick+1]))),float(np.max(abs(s-physics['smoothed_target'][tick+1])))])
        if tick==299:initial=clone.snapshot()
    assert np.max(errors)==0 and np.max(bankerrors)<1e-12,(name,np.max(errors),np.max(bankerrors))
    i=300;state=physical_state(np.r_[physics['position_world_xy'][i],.45],physics['root_quaternion_world_wxyz'][i],physics['linear_velocity_world_m_s'][i],physics['angular_velocity_world_rad_s'][i],physics['joint_position_rad'][i],physics['joint_velocity_rad_s'][i])
    # XY-only legacy root trace has no z; .45 is not used as measured height.
    # No height/terrain transfer is admitted. Scope remains original open plane.
    yaw=state['yaw'];c,s=np.cos(yaw),np.sin(yaw);R=np.array([[c,s],[-s,c]])
    xy=(physics['position_world_xy'][i:]-physics['position_world_xy'][i])@R.T
    e=make_entry(name,state,initial,xy,np.load(r/'requests.npz')['requests'][60:],physics['simulation_time_s'][i:]-physics['simulation_time_s'][i])
    entries.append(e)
    sources=[r/'physics.npz',r/'requests.npz',next((r/'feedback').glob('*.json')),r/'identity.json']
    evidence.append(dict(name=name,feedback_ticks=len(feedback),max_applied_smoothed_difference=0.,max_bank_difference=float(np.max(bankerrors)),initial_state_from='replayed official deterministic chain through 299th pastfeedback only; 6s before action',sources={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}))
def convert(v):
    if isinstance(v,np.ndarray):return v.tolist()
    raise TypeError(type(v))
(H/'guarded_response_library.json').write_text(json.dumps(entries,default=convert,indent=2)+'\n')
(H/'execution_reconstruction_audit.json').write_text(json.dumps(dict(rows=evidence,claim='measured feedback reconstruction exact; no recursive physical prediction'),indent=2)+'\n')
print(json.dumps(evidence,indent=2))
