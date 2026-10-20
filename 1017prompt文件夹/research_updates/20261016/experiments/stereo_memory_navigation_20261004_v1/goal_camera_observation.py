"""One camera-axis goal-bearing observation pulse; no pose truth or model."""
import numpy as np

class GoalCameraObservation:
    def __init__(self, initial_xy):
        self.initial_xy=np.asarray(initial_xy,float).copy()
        self.phase='await';self.last_source=-1;self.bad_epochs=0;self.start=None
        self.bank=0.;self.records=[];self.last_now=-np.inf

    def request(self, now, source_frame, source_time, point_count, axis_world, axis_time, root_xy, goal_xy, parent):
        if not np.isfinite(now) or not np.isfinite(source_time) or not np.isfinite(axis_time) or source_time>now or axis_time!=now or now<self.last_now:
            raise ValueError('current feedback and past source clock before payload')
        self.last_now=now
        if source_frame<self.last_source:raise ValueError('decreasing source epoch')
        axis=np.asarray(axis_world,float);root=np.asarray(root_xy,float);goal=np.asarray(goal_xy,float)
        if axis.shape!=(3,) or root.shape!=(2,) or goal.shape!=(2,) or not np.isfinite(axis).all() or not np.isfinite(root).all() or not np.isfinite(goal).all():
            raise ValueError('finite axis/root/known goal')
        defined=np.linalg.norm(axis[:2])>1e-8 and np.linalg.norm(goal-root)>1e-8
        error=float(np.arctan2(np.sin(np.arctan2(axis[1],axis[0])-np.arctan2(*(goal-root)[::-1])),np.cos(np.arctan2(axis[1],axis[0])-np.arctan2(*(goal-root)[::-1])))) if defined else None
        if source_frame>self.last_source:
            self.bad_epochs=self.bad_epochs+1 if point_count<100 else 0
            self.last_source=source_frame
        if self.phase=='await' and self.bad_epochs>=2:
            if not defined or not 0<error<np.pi/2:self.phase='inapplicable'
            else:self.phase='turn';self.start=now;self.bank=float(np.pi/6)
        if self.phase=='turn':
            # Four seconds = forty existing 10Hz request blocks; avoid float32 drift.
            blocks=round((now-self.start)/.1)
            if not defined or error<=0 or blocks>=40 or np.linalg.norm(root-self.initial_xy)>=.5:self.phase='hold'
        result=np.asarray(parent,float).copy() if self.phase=='await' else np.array([.5 if self.phase=='turn' else 0.,self.bank])
        self.records.append(dict(time_s=now,source_frame=int(source_frame),source_time_s=source_time,phase=self.phase,camera_goal_heading_error_rad=error,root_displacement_m=float(np.linalg.norm(root-self.initial_xy)),request=result.tolist(),once_only=True,no_future_plan=True))
        return result

def current_observer_inputs(now, native, source, extrinsic, goal_xy, parent):
    stamp=float(native['time_s'])
    if not np.isfinite(now) or not np.isfinite(stamp) or stamp!=now:
        raise ValueError('native prediction must equal independent current capture before pose payload')
    if not native['available'] or float(source['time_s'])>now:
        raise ValueError('available current native feedback and past source required')
    rotation=np.asarray(native['rotation_world'],float)
    position=np.asarray(native['position_world'],float)
    axis=rotation@np.asarray(extrinsic)[:3,2]
    return dict(time_s=now,source_frame=int(source['frame']),source_time_s=float(source['time_s']),point_count=int(source['points']),axis_world=axis.tolist(),axis_time=stamp,root_xy=position[:2].tolist(),goal_xy=np.asarray(goal_xy).tolist(),parent=np.asarray(parent).tolist())
