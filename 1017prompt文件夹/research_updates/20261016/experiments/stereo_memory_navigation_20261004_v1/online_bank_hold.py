"""Causal post-policy bank source during a zero-main request interval."""
import numpy as np

class OnlineBankHold:
    def __init__(self):
        self.previous_motion_aux=None
        self.active=False

    def snapshot(self):
        return dict(active=self.active,previous_motion_aux=None if self.previous_motion_aux is None else self.previous_motion_aux.tolist())

    def apply(self,request):
        action=np.asarray(request,np.float32).copy()
        if action.shape!=(10,) or not np.isfinite(action).all():raise ValueError('finite runtime request block required')
        entered=False
        if np.any(action[0::2]!=0):
            # This block is emitted unchanged; on the next call it is a past request.
            self.previous_motion_aux=action[1::2].copy()
            self.active=False
        elif self.previous_motion_aux is not None:
            entered=not self.active
            self.active=True
            action[1::2]=self.previous_motion_aux
        return action,dict(active=self.active,entered=entered,source='previous_emitted_motion_request' if self.active else 'unchanged_parent_request',state=self.snapshot())
