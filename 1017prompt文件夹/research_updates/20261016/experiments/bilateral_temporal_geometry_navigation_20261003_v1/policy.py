"""Map-free binocular temporal obstacle positions actually consume control."""
from collections import deque
import math
import time
import numpy as np
from benchmark.policies import GoalGreedyPolicy
from geometry import MultiViewGeometry
from control import observed_obstacles,front_blocked,choose_side,make_detour


class BilateralTemporalPolicy(GoalGreedyPolicy):
    policy_name='vision_bilateral_temporal_geometry_v1'
    privileged=False
    online_input_permissions=frozenset({'stereo_rgb','position_world_xy','yaw_world_rad',
                       'quaternion_world_wxyz_history','proprio_history','action_history','pointgoal_base_xy'})

    def __init__(self,protocol):
        super().__init__(protocol)
        self.records=[];self.geometry=MultiViewGeometry();self._cache=deque();self._path_xy=None
        self._path_s=None;self._index=0;self._blocks=0;self._consecutive=0;self.side=None
        self.last_prediction=None;self.last_observed=np.empty((0,2))

    def reset(self,context):
        super().reset(context)
        self.records.clear();self.geometry=MultiViewGeometry();self._cache.clear();self._path_xy=None
        self._path_s=None;self._index=0;self._blocks=0;self._consecutive=0;self.side=None
        self.last_prediction=None;self.last_observed=np.empty((0,2))

    def act(self,observation,pointgoal_base_xy):
        started=time.perf_counter()
        if observation.quaternion_world_wxyz_history is None:
            raise ValueError('declared current measured attitude required')
        position=np.asarray(observation.position_world_xy,dtype=float);yaw=float(observation.yaw_world_rad)
        goal=np.asarray(pointgoal_base_xy,dtype=float);c,s=math.cos(yaw),math.sin(yaw)
        goal_world=position+np.array([[c,-s],[s,c]])@goal
        t=self._blocks/10.
        pred=self.geometry.update(observation.stereo_left_rgb,observation.stereo_right_rgb,position,
                                 observation.quaternion_world_wxyz_history[-1],t)
        fresh={eye:observed_obstacles(pred[eye]['points_world'],position) for eye in ('left','right')}
        while self._cache and self._cache[0][0]<t-3.:self._cache.popleft()
        self._cache.append((t,np.vstack(list(fresh.values()))))
        observed=np.unique(np.vstack([p for _,p in self._cache]),axis=0)
        blocked=front_blocked(observed,position,goal_world)
        self._consecutive=self._consecutive+1 if blocked else 0
        choice_scores=None;created=False
        if self._path_xy is None and self._consecutive>=2:
            side,choice_scores=choose_side(observed,position,goal_world)
            path=make_detour(position,goal_world,side)
            if path is not None:
                self.side=side;self._path_xy=path
                self._path_s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
                created=True
        target=goal
        if self._path_xy is not None:
            self._index+=int(np.argmin(np.linalg.norm(self._path_xy[self._index:]-position,axis=1)))
            target_index=min(int(np.searchsorted(self._path_s,self._path_s[self._index]+1)),len(self._path_xy)-1)
            delta=self._path_xy[target_index]-position
            target=np.array([[c,s],[-s,c]])@delta
            if target_index==len(self._path_xy)-1:target=goal
        action=super().act(observation,target)
        self.last_prediction=pred;self.last_observed=observed.copy()
        self.records.append(dict(block=self._blocks,time_s=t,position_world_xy=position.tolist(),goal_base_xy=goal.tolist(),
             target_base_xy=target.tolist(),central_blocked=blocked,consecutive=self._consecutive,
             reference_created=created,reference_index=self._index,side=self.side,
             reference_source='observed_RGB_geometry' if self._path_xy is not None else 'unverified_nominal_PointGoal',
             side_observed_clearance=choice_scores,unknown_space_is_not_free=True,
             eyes={eye:dict(depth_predictions=len(pred[eye]['depth_m']),corners=pred[eye]['corners'],
                      support_views=pred[eye]['support_views'],obstacle_cells=len(fresh[eye])) for eye in ('left','right')},
             observed_obstacle_cells=len(observed),request_main_aux=action[:2].tolist(),
             inference_wall_s=time.perf_counter()-started))
        self._blocks+=1
        return action
