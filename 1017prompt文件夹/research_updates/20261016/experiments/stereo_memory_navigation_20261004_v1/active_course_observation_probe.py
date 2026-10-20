"""One bounded course-feedback observation; no recursive motion model or labels."""
import math
import numpy as np
from neutral_bank_visibility_probe import NeutralBankVisibilityProbe

class ActiveCourseObservationProbe(NeutralBankVisibilityProbe):
    def __init__(self):
        super().__init__();self.event_course=None;self.course_goal_reached=False
    def choose(self,now,source_frame,source_time,qualified_hazards,handed_off,nominal_request,velocity_time,velocity_world):
        t=float(now);st=float(source_time);vt=float(velocity_time)
        if not np.isfinite(t) or not np.isfinite(st) or t<self.time or st>t+1e-6 or t-st>.5+1e-6 or st<self.last_source_time:
            raise ValueError('causal source/current clock required before payload')
        if not np.isfinite(vt) or abs(vt-t)>1e-9:
            raise ValueError('current velocity clock required before payload')
        v=np.asarray(velocity_world,float)
        if v.shape!=(3,) or not np.isfinite(v).all():raise ValueError('finite current world velocity required')
        speed=float(np.linalg.norm(v[:2]));course=math.atan2(v[1],v[0]) if speed>=.08 else None
        old_event=self.event_time
        # Reuse the original causal/two-distinct-source event detector only.
        super().choose(t,source_frame,st,qualified_hazards,handed_off,nominal_request)
        if old_event is None and self.event_time is not None:self.event_course=course
        angle=None
        if self.event_course is not None and course is not None:
            angle=math.atan2(math.sin(course-self.event_course),math.cos(course-self.event_course))
            if angle>=math.radians(10):self.course_goal_reached=True
        active=self.event_time is not None and t<self.event_time+6.
        turning=active and self.event_course is not None and not self.course_goal_reached and t<self.event_time+4.
        action=np.asarray(nominal_request).copy()
        if active:
            action[:]=0
            if turning:action[::2]=.5;action[1::2]=-math.pi/6
        self.diagnostics.update(active=active,finished=self.event_time is not None and not active,
                                turning=turning,course_goal_reached=self.course_goal_reached,
                                event_course_rad=self.event_course,current_course_rad=course,
                                current_XY_speed_m_s=speed,measured_course_change_rad=angle,
                                turn_not_exercised_invalid_event_course=self.event_time is not None and self.event_course is None,
                                maximum_turn_s=4.,maximum_observation_s=6.,desired_bank_rad=-math.pi/8,
                                physical_response_prediction_used=False,no_source_gate_relaxation=True)
        return action
