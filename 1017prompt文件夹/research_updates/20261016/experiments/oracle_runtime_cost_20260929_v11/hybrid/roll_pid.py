"""Experimental physical-bank loop, outputs auxiliary position before formal smoother."""
import math
import numpy as np

def bank_from_quaternion(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:
        raise ValueError('expected finite nonzero wxyz quaternion')
    w,x,y,z=q/np.linalg.norm(q)
    return float(np.arcsin(np.clip(2*(w*x+y*z),-1,1)))

class BankPID:
    def __init__(self,kp=.8,ki=.1,kd=.35,dt=.02):
        self.kp,self.ki,self.kd,self.dt=kp,ki,kd,dt
        self.reset()
    def reset(self):
        self.integral_output=0.;self.previous_bank=None;self.rate=0.;self.last={}
    def step(self,target,bank,downstream_blocked=False):
        target=np.asarray(target,dtype=float)
        if target.shape!=(2,) or not np.isfinite(target).all() or not math.isfinite(bank):
            raise ValueError('nonfinite control state')
        bound=math.pi/6;ff=float(np.clip(target[1],-bound,bound));desired=.75*ff
        if self.previous_bank is not None:
            raw_rate=(bank-self.previous_bank)/self.dt
            self.rate+=self.dt/(.1+self.dt)*(raw_rate-self.rate)
        self.previous_bank=bank
        error=desired-bank
        candidate=float(np.clip(self.integral_output+self.ki*error*self.dt,-math.radians(5),math.radians(5)))
        trial=ff+self.kp*error+candidate-self.kd*self.rate
        if not downstream_blocked and (abs(trial)<=bound or trial*error<0):
            self.integral_output=candidate
        raw=ff+self.kp*error+self.integral_output-self.kd*self.rate
        u=float(np.clip(raw,-bound,bound))
        self.last=dict(bank_rad=bank,desired_bank_rad=desired,error_rad=error,bank_rate_rad_s=self.rate,integral_output_rad=self.integral_output,aux_feedforward_rad=ff,pid_raw_aux_rad=raw,pid_protocol_aux_rad=u,pid_clipped=abs(raw-u)>1e-9,downstream_blocked=bool(downstream_blocked))
        return np.array([target[0],u],dtype=np.float32)

class FeedbackSmoother:
    def __init__(self,inner,robot,pid,records):
        self.inner,self.robot,self.pid,self.records=inner,robot,pid,records
        self.blocked=False
    def __getattr__(self,name):return getattr(self.inner,name)
    def reset(self):
        self.pid.reset();self.blocked=False;self.inner.reset()
    def step(self,target,minimum,maximum,**kwargs):
        bank=bank_from_quaternion(self.robot.get_world_pose()[1])
        corrected=self.pid.step(target,bank,downstream_blocked=self.blocked)
        applied=self.inner.step(corrected,minimum,maximum,**kwargs)
        smoothed=np.asarray(getattr(self.inner,'last_smoothed_target',applied))
        self.blocked=abs(float(applied[1]-smoothed[1]))>1e-6
        self.records.append({**self.pid.last,'tick':len(self.records),'main_request_rad_s':float(target[0]),'smoothed_aux_rad':float(smoothed[1]),'applied_aux_rad':float(applied[1]),'supervisor_active':self.blocked})
        return applied

def install_runtime(base_class,kp=.8,ki=.1,kd=.35,output=None):
    """Return an isolated runtime class; stock execute still runs once per tick."""
    from pathlib import Path
    import json
    class BankRuntime(base_class):
        def reset(self,episode):
            result=super().reset(episode)
            self.bank_records=[]
            self._action_smoother=FeedbackSmoother(self._action_smoother,self._robot,BankPID(kp,ki,kd),self.bank_records)
            self.bank_episode_id=episode.episode_id
            return result
        def close(self,*args,**kwargs):
            if output is not None and hasattr(self,'bank_records'):
                dest=Path(output);dest.mkdir(parents=True,exist_ok=True)
                (dest/('bank_feedback_'+self.bank_episode_id+'.json')).write_text(json.dumps(self.bank_records))
            return super().close(*args,**kwargs)
    return BankRuntime
