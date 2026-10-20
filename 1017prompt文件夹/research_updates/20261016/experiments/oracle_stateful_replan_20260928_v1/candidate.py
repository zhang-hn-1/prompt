"""One-shot reference-source intervention; unchanged inherited B0 tracking."""
import time
import numpy as np
from probe import EventOracle

class ReplanOracle(EventOracle):
    def __init__(self,*args,replan_callback,**kwargs):
        super().__init__(*args,**kwargs)
        self.replan_callback=replan_callback
    def reset(self,context):
        super().reset(context)
        self.replan_attempted=False
        self.replan_result=None
        self.replan_seconds=None
        self.original_goal=self._reference_xy[-1].copy()
    def act(self,observation,pointgoal_base_xy):
        if not self.replan_attempted and self.ready:
            self.replan_attempted=True
            started=time.perf_counter()
            result=self.replan_callback(observation,self.original_goal.copy(),active_direction=self._active_drive_direction)
            self.replan_seconds=time.perf_counter()-started
            self.replan_result=result
            if result.status=='success' and len(result.waypoints)>=2:
                # Never reset the direction latch, stop counter, physical state,
                # or smoother. A new reverse request still uses B0 braking.
                self._reference=tuple(result.waypoints)
                self._reference_xy=np.asarray([(w.x_m,w.y_m) for w in self._reference])
                self._oracle_path_xy=self._reference_xy.copy()
                self._path_index=0
                self._off_reference_cycles=0
        return super().act(observation,pointgoal_base_xy)
