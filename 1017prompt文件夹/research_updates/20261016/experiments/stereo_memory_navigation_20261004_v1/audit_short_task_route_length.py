"""Offline 2D distance diagnostic, never planner/runtime input or formal SPL."""
from pathlib import Path
import numpy as np,json,hashlib,heapq,datetime
from scipy.ndimage import distance_transform_edt
H=Path(__file__).resolve().parent;o=H/'short_obstacle_current_run';inp=[o/'offline_occupancy_label.npz',o/'task_episode.json',Path(__file__).resolve()];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();protocol=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),files={str(p):sha(p) for p in inp},clearance_m=.6,grid_only_not_Dubins=True,no_online_use=True,formal_SLP_not_recomputed=True)
(H/'short_task_route_length_preanalysis.json').write_text(json.dumps(protocol,indent=2)+'\n');d=np.load(inp[0]);task=json.loads(inp[1].read_text())['actual_episode'];res=float(d['resolution']);origin=d['origin'];occ=d['labels']>0;clearance=distance_transform_edt(~occ)*res;free=clearance>.6;start=tuple(np.rint((np.array(task['start_position_world_m'][:2])-origin)/res).astype(int));goal=tuple(np.rint((np.array(task['goal_position_world_m'][:2])-origin)/res).astype(int));assert free[start] and free[goal];cost={start:0.};parent={};queue=[(0,start)];moves=[(dx,dy,np.hypot(dx,dy)*res) for dx in [-1,0,1] for dy in [-1,0,1] if dx or dy]
while queue:
 g,u=heapq.heappop(queue)
 if g!=cost[u]:continue
 if u==goal:break
 for dx,dy,l in moves:
  v=(u[0]+dx,u[1]+dy)
  if min(v)<0 or v[0]>=free.shape[0] or v[1]>=free.shape[1] or not free[v]:continue
  if dx and dy and (not free[u[0]+dx,u[1]] or not free[u[0],u[1]+dy]):continue
  n=g+l
  if n<cost.get(v,np.inf):cost[v]=n;parent[v]=u;heapq.heappush(queue,(n,v))
assert goal in cost;path=[goal]
while path[-1]!=start:path.append(parent[path[-1]])
xy=origin+np.array(path[::-1])*res;result=dict(protocol=protocol,span_m=2.6,point_grid_path_m=float(cost[goal]),min_path_y_m=float(xy[:,1].min()),path_xy=xy.tolist(),nominal_required_after_6s_m_s=float(cost[goal]/29),main_request_rad_s=.3,nominal_no_slip_radius_times_request_m_s=.4*.3,not_a_physical_XY_speed_bound=True,not_formal_SLP=True,task_is_shorter_goal_span_not_guaranteed_short_detour=True,original_rules_not_changed=True)
(H/'short_task_route_length_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k not in ['path_xy','protocol']})
