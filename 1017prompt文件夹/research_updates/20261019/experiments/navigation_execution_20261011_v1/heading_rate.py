"""Coordinate-consistent outer heading-rate feedback; original tracker retained."""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def projected_heading_rate(quaternion_wxyz, angular_velocity_world):
    q=np.asarray(quaternion_wxyz,float)
    w=np.asarray(angular_velocity_world,float)
    if q.shape!=(4,) or w.shape!=(3,) or not np.isfinite(q).all() or not np.isfinite(w).all():
        raise ValueError('finite current quaternion4 and world angular velocity3 required')
    if abs(float(np.linalg.norm(q))-1.)>1e-5:
        raise ValueError('current unit quaternion required')
    axis=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()[:,0]
    horizontal=float(axis[0]**2+axis[1]**2)
    out=dict(supported=horizontal>1e-8,horizontal_axis_norm_squared=horizontal,
             world_z_rate_rad_s=float(w[2]),yaw_rate_rad_s=None,heading_rad=None)
    if out['supported']:
        derivative=np.cross(w,axis)
        out.update(yaw_rate_rad_s=float((axis[0]*derivative[1]-axis[1]*derivative[0])/horizontal),
                   heading_rad=float(math.atan2(axis[1],axis[0])))
    return out


def with_heading_rate_tracker(base, *, enabled):
    class ProjectedHeadingRateTracker(base):
        _heading_rate_current=None

        def act(self, observation, goal):
            if self._heading_rate_current is not None:
                raise RuntimeError('reentrant tracker act forbidden')
            self._heading_rate_current=observation
            try:
                return super().act(observation,goal)
            finally:
                self._heading_rate_current=None

        def _begin_tracking_audit(self, observation):
            super()._begin_tracking_audit(observation)
            self._tracking_audit.update(heading_rate_consumer_enabled=bool(enabled),
                heading_rate_consumer_applied=False,heading_rate_geometry_supported=None)

        def _request_action(self, main_velocity, auxiliary_angle, *, source):
            audit=self._tracking_audit
            if source=='tracking' and audit.get('valid',False):
                observation=self._heading_rate_current
                if observation is None:
                    raise RuntimeError('current inherited tracking call required')
                geometry=projected_heading_rate(observation.quaternion_world_wxyz_history[-1],
                                               observation.proprio_history[-1,3:6])
                old=float(audit['yaw_rate_feedback_auxiliary_rad'])
                audit.update(heading_rate_geometry_supported=geometry['supported'],
                    measured_projected_heading_rate_rad_s=geometry['yaw_rate_rad_s'],
                    heading_rate_feedback_before_rad=old,
                    heading_rate_feedback_after_rad=old,
                    heading_rate_horizontal_axis_norm_squared=geometry['horizontal_axis_norm_squared'])
                if geometry['supported']:
                    difference=(geometry['heading_rad']-float(observation.yaw_world_rad)+math.pi)%(2*math.pi)-math.pi
                    if abs(difference)>1e-6:
                        raise ValueError('current quaternion and navigation heading mismatch')
                    sign=1. if main_velocity>0 else -1.
                    new=float(np.clip(-sign*self.yaw_rate_feedback_gain_s*(
                        float(audit['desired_world_yaw_rate_rad_s'])-geometry['yaw_rate_rad_s']),
                        math.radians(-5.),math.radians(5.)))
                    audit['heading_rate_feedback_candidate_rad']=new
                    if enabled:
                        auxiliary_angle=float(auxiliary_angle)-old+new
                        audit.update(yaw_rate_feedback_auxiliary_rad=new,
                            heading_rate_feedback_after_rad=new,
                            heading_rate_consumer_applied=not math.isclose(new,old,rel_tol=0.,abs_tol=1e-12),
                            v1_model_proxy_fields_scope='before_projected_heading_rate_consumer')
            return super()._request_action(main_velocity,auxiliary_angle,source=source)
    return ProjectedHeadingRateTracker


def with_heading_rate_policy(base, *, enabled):
    class ProjectedHeadingRatePolicy(base):
        def reset(self, context):
            super().reset(context)
            self.tracker.__class__=with_heading_rate_tracker(type(self.tracker),enabled=enabled)

        def act(self, observation, goal):
            count=len(self.tracker.tracking_records)
            action=super().act(observation,goal)
            fresh=len(self.tracker.tracking_records)>count
            self.records[-1].update(heading_rate_enabled=bool(enabled),
                heading_rate_fresh_tracker_record=fresh,
                heading_rate_tracking=None if not fresh else dict(self.tracker.tracking_diagnostics))
            return action
    return ProjectedHeadingRatePolicy
