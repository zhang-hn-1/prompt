"""Use original hard_0 truth map, not the all-free diagnostic fixture."""
import dataclasses,hashlib,json,math
import numpy as np
from probe import ROOT,OLD
from benchmark.task_cards import load_occupancy_npz
from benchmark.grid_planning import inflate_obstacles
from benchmark.kinodynamic_planning import KinodynamicPlanRequest,plan_kinodynamic_path
from collection_schema import load_contract

class SourceMapReplanner:
    def __init__(self,case,model,planner=plan_kinodynamic_path):
        self.model=model;self.planner=planner
        config=json.loads((OLD/f'episode_hard_0_e1_{case}'/'config.json').read_text())
        first=config['reference']['source_reference_pose_first']
        self.origin=np.array(first[:2]);self.yaw=float(first[2])
        self.rotation=np.array([[math.cos(self.yaw),-math.sin(self.yaw)],[math.sin(self.yaw),math.cos(self.yaw)]])
        path=ROOT/'research/ball_wm/data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz'
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if digest!='547dd73c52b45f9cc1aa8dca561c779349391f6ecef06ef59026921284b467da':raise ValueError('original map hash mismatch')
        self.map=load_occupancy_npz(path)
        contract=load_contract(ROOT/'research/ball_wm/configs/data_contract.yaml')
        self.grid=inflate_obstacles(np.where(self.map.grid==contract.bev.free,0,1).astype(np.uint8),radius_m=.5,resolution_m=self.map.resolution_m,occupied=1)
        self.provenance=dict(path=str(path),sha256=digest,inflation_radius_m=.5,frame='original scene world',node_budget=5000)
    def __call__(self,obs,goal):
        start=self.rotation@obs.position_world_xy+self.origin
        end=self.rotation@goal+self.origin
        req=KinodynamicPlanRequest(occupancy_grid=self.grid,resolution_m=self.map.resolution_m,origin_xy=tuple(self.map.global_origin_xy),start_xy=tuple(start),start_yaw_rad=obs.yaw_world_rad+self.yaw,goal_xy=tuple(end),node_budget=5000)
        result=self.planner(req,self.model)
        waypoints=[]
        for w in result.waypoints:
            xy=self.rotation.T@(np.array([w.x_m,w.y_m])-self.origin)
            waypoints.append(dataclasses.replace(w,x_m=float(xy[0]),y_m=float(xy[1]),yaw_rad=float((w.yaw_rad-self.yaw+math.pi)%(2*math.pi)-math.pi)))
        return dataclasses.replace(result,waypoints=tuple(waypoints))
