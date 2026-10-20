from pathlib import Path
import sys,json,hashlib,numpy as np,cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
h=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_margin_limit_20260929_v5');wm=Path('/home/an/project/ball_wm_project_full/.worktrees/e1-detour-benchmark/research/ball_wm');sys.path[:0]=[str(h),str(wm),str(wm/'scripts')]
from roll_pid import BankPID
from benchmark.task_cards import load_occupancy_npz
from collection_schema import load_contract
from scripts.bev_ground_truth import crop_global_occupancy,obstacle_clearance
bev=load_contract(wm/'configs/data_contract.yaml').bev;occ=load_occupancy_npz(wm/'data/benchmark_v1/occupancy/cluttered_hard_0_bodyband.npz');obstacle=np.argwhere(occ.grid==1)*occ.resolution_m+occ.global_origin_xy;tree=cKDTree(obstacle)
def project(xy,ref):
 a=ref[:-1,:2];d=np.diff(ref[:,:2],axis=0);den=np.maximum((d*d).sum(axis=1),1e-12);u=np.clip(((xy[:,None,:]-a)*d).sum(axis=2)/den,0,1);q=a+u[:,:,None]*d;dist=np.linalg.norm(xy[:,None,:]-q,axis=2);i=dist.argmin(axis=1);r=np.arange(len(xy));return dist[r,i]
rows=[];videos=[];fig,axs=plt.subplots(2,3,figsize=(15,10));manifest=json.loads((wm/'configs/benchmark_v1/hard_0_dev_manifest.json').read_text());goals={e['episode_id']:e['goal_position_world_m'][:2] for e in manifest['episodes']}
for row,case in enumerate(['0002','0004']):
 eid=f'hard_0_e1_{case}';n=h/f'navigation_margin_{case}';p=n/f'episode_{eid}';b=h.parent/f'oracle_pid_validation_20260929_v4/navigation_hard_0/episode_{eid}';z=np.load(p/'states.npz');bz=np.load(b/'states.npz');m=json.loads((p/'metrics.json').read_text());bm=json.loads((b/'metrics.json').read_text());runtime=json.loads((n/'feedback'/f'runtime_audit_{eid}.json').read_text());cfg=runtime['effective_config'];assert cfg['safety_margin_m']==.2 and cfg['ball_radius_m']==.4 and cfg['auxiliary_velocity_limit_rad_s']==.3
 pose=np.load(h/f'plan_{eid}_0.2.npz')['pose'];assert np.array_equal(pose,z['kinodynamic_reference_pose'])
 initial={k:bool(np.array_equal(z[k][0],bz[k][0])) for k in ['position_world_xy','yaw_world_rad','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action','roll_world_rad']};assert all(initial.values())
 fb=json.loads((n/'feedback'/f'bank_feedback_{eid}.json').read_text());pid=BankPID(kp=.4,ki=.1,kd=.7);blocked=False;errs=[]
 for r in fb:
  pid.step([r['main_request_rad_s'],r['aux_feedforward_rad']],r['bank_rad'],downstream_blocked=blocked);errs.append(abs(pid.last['pid_protocol_aux_rad']-r['pid_protocol_aux_rad']));blocked=r['supervisor_active']
 assert max(errs)<1e-12 and len(fb)==5*m['action_blocks']
 checks=[]
 for source,zz in [('old',bz),('new',z)]:
  xy=zz['position_world_xy'];ref=zz['kinodynamic_reference_pose'];tt=zz['simulation_time_s'];distance=np.linalg.norm(xy-np.asarray(goals[eid]),axis=1);err=project(xy,ref);cl=np.array([obstacle_clearance(crop_global_occupancy(occ.grid,occ.global_origin_xy,x,y,bev),.4,bev) for x,y in zip(xy,zz['yaw_world_rad'])]);assert np.allclose(cl,zz['clearance_m'],atol=1e-6)
  dense=np.concatenate([np.linspace(a[:2],bb[:2],max(2,int(np.ceil(np.linalg.norm(bb[:2]-a[:2])/.02))+1)) for a,bb in zip(ref[:-1],ref[1:])]);global_ref_min=float((tree.query(dense)[0]-.4).min());checks.append(dict(group=source,geometric_error_rms_m=float(np.sqrt(np.mean(err**2))),error_peak_m=float(err.max()),error_final_m=float(err[-1]),reference_length_m=float(np.linalg.norm(np.diff(ref[:,:2],axis=0),axis=1).sum()),reference_global_cell_min_clearance_m=global_ref_min,final_goal_distance_m=float(distance[-1]),actual_path_length_m=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()),physical_contacts=int(zz['physical_contact'].sum()),final_formal_clearance_m=float(cl[-1])))
  color='tab:blue' if source=='old' else 'tab:orange';axs[row,0].plot(ref[:,0],ref[:,1],'--',color=color,label=source+' reference');axs[row,0].plot(xy[:,0],xy[:,1],color=color,label=source+' actual');axs[row,1].plot(tt,distance,color=color,label=source);axs[row,2].plot(tt,cl,color=color,label=source+' clearance')
 keys=['success','collision','timeout','runtime_error','elapsed_s','final_distance_m','path_length_m','minimum_clearance_m','termination_reason'];rows.append(dict(case=case,baseline={k:bm.get(k) for k in keys},candidate={k:m.get(k) for k in keys},same_initial=initial,reference_matches_precomputed_margin20=True,reference_changed=not np.array_equal(bz['kinodynamic_reference_pose'],pose),audit=checks,pid_ticks=len(fb),pid_max_error=max(errs),effective_config=cfg,contact_available_all=all(runtime['contact_available_samples']),contact_available_sample_count=len(runtime['contact_available_samples'])))
 allp=np.vstack((bz['kinodynamic_reference_pose'][:,:2],pose[:,:2],z['position_world_xy']));lo=allp.min(axis=0)-.7;hi=allp.max(axis=0)+.7;mask=np.all((obstacle>=lo)&(obstacle<=hi),axis=1);axs[row,0].scatter(obstacle[mask,0],obstacle[mask,1],s=7,color='.8',marker='s',zorder=-1);axs[row,0].set_xlim(lo[0],hi[0]);axs[row,0].set_ylim(lo[1],hi[1]);axs[row,0].set_aspect('equal');axs[row,0].set_title(case+' planning margin .10 vs .20');axs[row,1].set_title(case+' distance to same goal');axs[row,2].set_title(case+' formal collision clearance');axs[row,2].axhline(0,color='red')
 for ax in axs[row]:ax.grid();ax.legend(fontsize=8)
 for v in p.glob('*.mp4'):
  cap=cv2.VideoCapture(str(v));frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS);first,_=cap.read();cap.set(cv2.CAP_PROP_POS_FRAMES,frames-1);last,_=cap.read();cap.release();assert first and last and fps>0 and frames>0;videos.append(dict(path=str(v),frames=frames,fps=fps,first_decodes=first,last_decodes=last))
fig.tight_layout();fig.savefig(h/'margin_navigation.png',dpi=160);(h/'margin_navigation_assessment.json').write_text(json.dumps(rows,indent=2));(h/'video_verification.json').write_text(json.dumps(videos,indent=2));identity=json.loads((h/'navigation_source_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items());print(json.dumps([{k:r[k] for k in ['case','candidate','audit','contact_available_all','pid_ticks']} for r in rows],indent=2))
