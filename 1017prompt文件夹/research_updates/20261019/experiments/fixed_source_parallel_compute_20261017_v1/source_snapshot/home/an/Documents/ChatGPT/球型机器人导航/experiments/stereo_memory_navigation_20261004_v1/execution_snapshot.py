"""Read-only snapshots of the actual PID/protocol/smoother execution chain.

No physics response is predicted here. step takes explicitly current measured
feedback; recursively predicting that feedback is a separate unaccepted layer.
"""
import copy,sys
from pathlib import Path
import numpy as np
P=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm')
sys.path.insert(0,str(P))
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'oracle_feasible_route_20260929_v9/gated_speed_012'))
from scripts.action_smoothing import ActionSmoother,ActionSmootherSnapshot
from training.action_protocol import ActionProtocol
from roll_pid import BankPID

SCHEMA='frozen_bank_pid_execution_snapshot_v1'
def export_execution(wrapper,protocol):
    inner=wrapper.inner;pid=wrapper.pid
    return dict(schema=SCHEMA,protocol=dict(protocol_id=protocol.protocol_id,minimum=list(protocol.minimum),maximum=list(protocol.maximum),controls_per_block=protocol.controls_per_block),
        inner_config=dict(dt_s=inner.dt_s,max_main_acceleration_rad_s2=inner.max_main_acceleration_rad_s2,max_auxiliary_rate_rad_s=inner.max_auxiliary_rate_rad_s,max_auxiliary_velocity_rad_s=inner.max_auxiliary_velocity_rad_s),
        inner=dict(current_action=list(inner.snapshot().current_action),last_smoothed_target=list(inner.snapshot().last_smoothed_target)),
        pid_config=dict(kp=pid.kp,ki=pid.ki,kd=pid.kd,dt=pid.dt),
        pid=dict(integral_output=pid.integral_output,previous_bank=pid.previous_bank,rate=pid.rate,last=copy.deepcopy(pid.last)),blocked=bool(wrapper.blocked))

class ExecutionClone:
    @classmethod
    def from_snapshot(cls,snapshot):
        s=copy.deepcopy(snapshot)
        if s['schema']!=SCHEMA:raise ValueError('execution schema mismatch')
        obj=cls();obj.protocol=ActionProtocol(**s['protocol']);obj.inner=ActionSmoother(**s['inner_config'])
        obj.inner.restore(ActionSmootherSnapshot(**s['inner']))
        obj.pid=BankPID(**s['pid_config'])
        for key in ('integral_output','previous_bank','rate','last'):setattr(obj.pid,key,s['pid'][key])
        numbers=[obj.pid.integral_output,obj.pid.rate]
        if obj.pid.previous_bank is not None:numbers.append(obj.pid.previous_bank)
        if not np.isfinite(numbers).all():raise ValueError('nonfinite PID state')
        obj.blocked=bool(s['blocked']);return obj
    def snapshot(self):return export_execution(self,self.protocol)
    def step(self,raw_request,*,bank_rad,aux_position_rad,aux_velocity_rad_s):
        # Formal runtime clips raw float32 BEFORE feedback PID; no applied truth input.
        raw=np.asarray(raw_request,np.float32)
        block=np.tile(raw,self.protocol.controls_per_block)
        target=self.protocol.clip_block(block).reshape(-1,2)[0]
        corrected=self.pid.step(target,float(bank_rad),downstream_blocked=self.blocked)
        kwargs={}
        if self.inner.max_auxiliary_velocity_rad_s is not None:
            kwargs=dict(auxiliary_position_rad=float(aux_position_rad),auxiliary_velocity_rad_s=float(aux_velocity_rad_s))
        applied=self.inner.step(corrected,self.protocol.minimum,self.protocol.maximum,**kwargs)
        smooth=self.inner.last_smoothed_target
        self.blocked=abs(float(applied[1]-smooth[1]))>1e-6
        return applied,np.asarray(smooth,np.float32)
