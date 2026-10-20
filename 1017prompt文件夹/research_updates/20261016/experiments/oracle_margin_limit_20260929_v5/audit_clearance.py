from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from benchmark.task_cards import load_occupancy_npz
from collection_schema import load_contract
from scripts.bev_ground_truth import crop_global_occupancy,obstacle_clearance
h=Path(__file__).resolve().parent;wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');old=h.parent/'oracle_pid_validation_20260929_v4/navigation_hard_0';bev=load_contract(wm/'configs/data_contract.yaml').bev
occ=load_occupancy_npz(wm/'data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz');points=np.argwhere(occ.grid==bev.occupied)*occ.resolution_m+occ.global_origin_xy;tree=cKDTree(points)
def exact(xy,yaw):return np.array([obstacle_clearance(crop_global_occupancy(occ.grid,occ.global_origin_xy,p,y,bev),.4,bev) for p,y in zip(xy,yaw)])
def project(xy,ref):
 a=ref[:-1,:2];d=np.diff(ref[:,:2],axis=0);den=np.maximum(np.sum(d*d,axis=1),1e-12);u=np.clip(((xy[:,None,:]-a)*d).sum(axis=2)/den,0,1);q=a+u[:,:,None]*d;dist=np.linalg.norm(xy[:,None,:]-q,axis=2);i=dist.argmin(axis=1);r=np.arange(len(xy));return q[r,i],i,u[r,i],dist[r,i]
rows=[];fig,axs=plt.subplots(2,3,figsize=(16,10))
for row,case in enumerate(['0002','0004']):
 z=np.load(old/f'episode_hard_0_e1_{case}/states.npz');xy=z['position_world_xy'];ref=z['kinodynamic_reference_pose'];t=z['simulation_time_s'];yaw=z['yaw_world_rad'];reconstructed=exact(xy,yaw);assert np.allclose(reconstructed,z['clearance_m'],atol=1e-6)
 projected,i,u,error=project(xy,ref);ry=ref[i,2]+u*np.arctan2(np.sin(ref[i+1,2]-ref[i,2]),np.cos(ref[i+1,2]-ref[i,2]));ref_same=exact(projected,ry);global_actual=tree.query(xy)[0]-.4;global_ref=tree.query(projected)[0]-.4
 dense=[]
 for a,b in zip(ref[:-1],ref[1:]):
  alpha=np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(b[:2]-a[:2])/.02))+1));dense.extend(np.column_stack((a[:2]+alpha[:,None]*(b[:2]-a[:2]),a[2]+alpha*np.arctan2(np.sin(b[2]-a[2]),np.cos(b[2]-a[2])))))
 dense=np.asarray(dense);denseclear=exact(dense[:,:2],dense[:,2]);dc=tree.query(dense[:,:2])[0]-.4
 result=dict(case=case,resolution_m=occ.resolution_m,formal_reconstruction_max_error=float(abs(reconstructed-z['clearance_m']).max()),collision_time_s=float(t[-1]),actual_final_clearance_m=float(reconstructed[-1]),actual_final_global_cell_clearance_m=float(global_actual[-1]),projected_reference_final_clearance_m=float(ref_same[-1]),projected_reference_global_cell_clearance_m=float(global_ref[-1]),final_cross_track_m=float(error[-1]),reference_min_dense_local_clearance_m=float(denseclear.min()),reference_min_dense_global_cell_clearance_m=float(dc.min()),last3s_peak_cross_track_m=float(error[t>=t[-1]-3].max()),max_local_global_disagreement_m=float(abs(reconstructed-global_actual).max()),physical_contact_samples=int(z['physical_contact'].sum()),physical_contact_availability='not serialized in archived V4; zero samples not proof of no contacts',collision_xy=xy[-1].tolist(),nearest_reference_xy=projected[-1].tolist());rows.append(result)
 np.savez(h/f'clearance_{case}.npz',time=t,actual_xy=xy,reference=ref,projected_reference=projected,actual_local_clearance=reconstructed,projected_local_clearance=ref_same,actual_global_clearance=global_actual,projected_global_clearance=global_ref,cross_track=error,dense_reference=dense,dense_local_clearance=denseclear)
 for ax,zoom in zip(axs[row,:2],[False,True]):
  p=xy[-1];allp=np.vstack((xy,ref[:,:2]));lo=(p-1.5 if zoom else allp.min(axis=0)-.8);hi=(p+1.5 if zoom else allp.max(axis=0)+.8);mask=np.all((points>=lo)&(points<=hi),axis=1);ax.scatter(points[mask,0],points[mask,1],s=9,marker='s',color='.65',label='occupied cell centers');ax.plot(ref[:,0],ref[:,1],'k--',label='reference');ax.plot(xy[:,0],xy[:,1],color='green',label='actual center');ax.add_patch(Circle(p,.4,fill=False,color='red',label='robot R=0.4m'));ax.add_patch(Circle(projected[-1],.4,fill=False,color='blue',linestyle=':',label='reference footprint'));ax.plot([p[0],projected[-1,0]],[p[1],projected[-1,1]],'r:');ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_aspect('equal');ax.set_title(case+(' final zoom' if zoom else ' full reference'));ax.grid();ax.set_xlabel('world X [m]');ax.set_ylabel('world Y [m]')
 ax=axs[row,2];ax.plot(t,reconstructed,label='actual formal clearance');ax.plot(t,ref_same,label='projected ref clearance');ax.plot(t,error,label='geometric tracking error');ax.axhline(0,color='r');ax.axhline(.1,color='.5',ls=':');ax.set_title(case+' distance budget');ax.set_xlabel('simulation time [s]');ax.set_ylabel('m');ax.grid();ax.legend(fontsize=8)
handles,labels=axs[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='upper center',ncol=5,fontsize=9);fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(h/'clearance_audit.png',dpi=160);(h/'clearance_audit.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
