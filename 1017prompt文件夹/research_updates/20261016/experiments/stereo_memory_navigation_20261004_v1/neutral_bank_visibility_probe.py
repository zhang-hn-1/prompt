"""One event-driven4s sensing interval; no models, geometry or future payloads."""
import numpy as np

class NeutralBankVisibilityProbe:
    def __init__(self):
        self.time=-np.inf;self.last_source_frame=None;self.last_source_time=-np.inf
        self.deficient_updates=0;self.event_time=None;self.event_frame=None
        self.diagnostics={}
    def choose(self,now,source_frame,source_time,qualified_hazards,handed_off,nominal_request):
        t=float(now);st=float(source_time)
        if not np.isfinite(t) or not np.isfinite(st) or t<self.time or st>t+1e-6 or t-st>.5+1e-6 or st<self.last_source_time:
            raise ValueError('causal source/current clock required before action')
        if not np.isfinite(source_frame) or int(source_frame)!=source_frame or source_frame<0:
            raise ValueError('nonnegative integer source frame required')
        if not np.isfinite(qualified_hazards) or int(qualified_hazards)!=qualified_hazards or qualified_hazards<0:
            raise ValueError('nonnegative integer qualified count required')
        frame=int(source_frame)
        if self.last_source_frame is not None and (frame<self.last_source_frame or (frame==self.last_source_frame and st!=self.last_source_time) or (frame>self.last_source_frame and st<=self.last_source_time)):
            raise ValueError('monotonic source frame/clock identity required')
        action=np.asarray(nominal_request).copy()
        if action.shape!=(10,) or not np.isfinite(action).all():raise ValueError('finite10D nominal request required')
        new=frame!=self.last_source_frame
        if new and self.event_time is None:
            if handed_off:
                self.deficient_updates=self.deficient_updates+1 if qualified_hazards<100 else 0
                if self.deficient_updates>=2:self.event_time=t;self.event_frame=frame
            else:self.deficient_updates=0
        active=self.event_time is not None and t<self.event_time+4.
        self.time=t;self.last_source_time=st;self.last_source_frame=frame
        self.diagnostics=dict(time_s=t,source_frame=frame,source_time_s=st,new_source_update=new,
                              qualified_current_hazards=int(qualified_hazards),deficient_updates=self.deficient_updates,
                              event_time_s=self.event_time,event_frame=self.event_frame,active=active,
                              finished=self.event_time is not None and not active,
                              nominal_request=action.tolist(),no_new_source_or_free_admission=True)
        return np.zeros_like(action) if active else action
