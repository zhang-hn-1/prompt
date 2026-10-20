from pathlib import Path
import sys,json,numpy as np
from scipy.spatial import cKDTree
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(wm),str(wm/'scripts')]
from benchmark.task_cards import load_occupancy_npz
from collection_schema import load_contract
occ=load_occupancy_npz(wm/'data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz');centers=np.argwhere(occ.grid==1)*occ.resolution_m+occ.global_origin_xy;tree=cKDTree(centers);half=occ.resolution_m/2

def squares(xy):
 # Any closer square center lies within nearest-center distance + half diagonal.
 distances,nearest=tree.query(xy);out=[]
 for p,d in zip(xy,distances):
  ids=tree.query_ball_point(p,float(d+np.sqrt(2)*half+1e-9));delta=np.maximum(abs(centers[ids]-p)-half,0);out.append(np.linalg.norm(delta,axis=1).min()-.4)
 return np.array(out)

def first(t,mask):
 ids=np.flatnonzero(mask);return float(t[ids[0]]) if len(ids) else None
rows=[]
sources=[h.parent/'oracle_pid_validation_20260929_v4/navigation_hard_0',h.parent/'oracle_request_chain_20260929_v6/navigation_course_0002',h.parent/'oracle_request_chain_20260929_v6/navigation_course_0004',h/'navigation_contact_0004']
for root in sources:
 for p in sorted(root.glob('episode_*/states.npz')):
  z=np.load(p);xy=z['position_world_xy'];center=tree.query(xy)[0]-.4;square=squares(xy);bound=center-np.sqrt(2)*half;assert np.all(square>=bound-1e-9) and np.all(square<=center+1e-9);t=z['simulation_time_s'];contact=z['physical_contact'].astype(bool);row=dict(run=root.name,episode=p.parent.name,formal_first_collision_s=first(t,z['collision'].astype(bool)),physical_first_contact_s=first(t,contact),cell_square_first_violation_s=first(t,square<=0),conservative_bound_first_violation_s=first(t,bound<=0),physical_contact_samples=int(contact.sum()),contacts_missed_by_formal=int(np.sum(contact&(z['clearance_m']>0))),contacts_missed_by_cell_squares=int(np.sum(contact&(square>0))),square_min_m=float(square.min()),old_formal_min_m=float(z['clearance_m'].min()));rows.append(row)
  if root==h/'navigation_contact_0004':np.savez(h/'clearance_crosscheck.npz',time=t,formal=z['clearance_m'],global_center=center,cell_square=square,conservative_bound=bound,physical_contact=contact,xy=xy)
(h/'conservative_audit.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
