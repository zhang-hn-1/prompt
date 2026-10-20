"""Single geometry action selector; only supplied information differs."""
import numpy as np
from benchmark.policies import geometric_tracker_action
from local_map import BeliefGrid,DirectionLatch

class InformationPolicy:
    uses_shared_tracker=True
    def __init__(self,protocol,mode,representation='points'):
        if mode not in ('map','visible','rgb'):raise ValueError(mode)
        if representation not in ('points','surface'):raise ValueError(representation)
        self.representation=representation
        self.protocol=protocol;self.mode=mode
        self.policy_name='information_diagnostic_'+mode
        self.privileged=mode!='rgb'
        self.online_input_permissions=frozenset({'current_pose','pointgoal','current_proprio',mode})
        self.reset(None)
    def reset(self,context):
        # Never consume runtime oracle reference, even for the map condition.
        self.belief=None;self._packet=None;self.records=[];self._path_xy=None
        self.latch=DirectionLatch();self._last_plan=-np.inf;self._reason='uninitialized'
    def set_current_packet(self,packet):
        if self._packet is not None:raise RuntimeError('unconsumed packet')
        if self.mode!='map' and 'known_grid' in packet:raise PermissionError('map forbidden in visible/RGB conditions')
        self._packet=packet
    def act(self,observation,goal):
        if self._packet is None:raise RuntimeError('current packet required')
        packet=self._packet;self._packet=None
        xy=np.asarray(observation.position_world_xy);yaw=float(observation.yaw_world_rad)
        time_s=float(packet['time_s']);root=np.asarray(packet.get('root_xyz',np.r_[xy,.4]))
        if self.belief is None:self.belief=BeliefGrid(xy-20.)
        if 'known_grid' in packet:
            known=packet['known_grid'];i,j=np.indices(self.belief.labels.shape)
            w=self.belief.origin+np.stack((i,j),axis=-1)*self.belief.resolution
            cells=np.rint((w-np.asarray(known['origin']))/known['resolution']).astype(int)
            labels=np.asarray(known['labels']);valid=(cells[...,0]>=0)&(cells[...,0]<labels.shape[0])&(cells[...,1]>=0)&(cells[...,1]<labels.shape[1])
            self.belief.labels[valid]=labels[cells[...,0][valid],cells[...,1][valid]]
        else:self.belief.update_points(packet['points_world'],root,time_s,
            ground_meshes=packet.get('ground_meshes',()) if self.representation=='surface' else ())
        self.belief.seed_body(xy)
        c,s=np.cos(yaw),np.sin(yaw);R=np.array([[c,-s],[s,c]])
        worldgoal=xy+R@np.asarray(goal)
        if time_s-self._last_plan>=.5-1e-6 or self._path_xy is None:
            self._path_xy,self._reason=self.belief.plan(xy,worldgoal);self._last_plan=time_s
        target=None;action=np.zeros(self.protocol.action_block_dim,dtype=np.float32)
        if self._path_xy is not None:
            path=self._path_xy;near=int(np.argmin(np.linalg.norm(path-xy,axis=1)))
            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path[near:],axis=0),axis=1))]
            index=min(len(path)-1,near+int(np.searchsorted(arc,1.)))
            target=R.T@(path[index]-xy);desired=1 if target[0]>=0 else -1
            direction=self.latch.request(desired,float(packet.get('speed_xy',0.)))
            if direction:action=direction*geometric_tracker_action(self.protocol,direction*target)
        self.records.append(dict(time_s=time_s,position_world_xy=xy.tolist(),route_reason=self._reason,
            representation=self.representation,
            target_base_xy=None if target is None else target.tolist(),request=action.tolist(),
            speed_xy=float(packet.get('speed_xy',0.)),direction=self.latch.direction,low_speed_count=self.latch.low,
            known_free=int((self.belief.labels==0).sum()),known_occupied=int((self.belief.labels==1).sum()),
            unknown=int((self.belief.labels==-1).sum()),points_received=len(packet['points_world'])))
        return action
