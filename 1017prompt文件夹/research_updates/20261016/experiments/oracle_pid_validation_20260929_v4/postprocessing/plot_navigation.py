from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
h=Path('/home/an/Documents/ChatGPT/球型机器人导航/experiments/oracle_pid_validation_20260929_v4');old=h.parent/'oracle_long_hold_navigation_20260928_v1';rows=[]
def geometric_error(xy,ref):
 a=ref[:-1,:2];d=ref[1:,:2]-a;den=np.sum(d*d,axis=1);den=np.maximum(den,1e-12)
 u=np.clip(np.sum((xy[:,None,:]-a)*d,axis=2)/den,0,1);dist=np.linalg.norm(xy[:,None,:]-(a+u[:,:,None]*d),axis=2)
 return dist.min(axis=1)
for suite in ['golden','hard_0']:
 folders=[p for p in sorted((h/('navigation_'+suite)).glob('episode_*')) if p.is_dir() and not p.name.endswith('.partial')]
 if not folders:continue
 fig,axes=plt.subplots(2,4 if suite=='hard_0' else 2,figsize=(15 if suite=='hard_0' else 10,8));axes=axes.ravel()
 for ax,p in zip(axes,folders):
  b=old/('navigation_'+suite)/p.name;v1=h.parent/'oracle_bank_mapping_20260929_v2'/('navigation_'+suite)/p.name;v1m=json.loads((v1/'metrics.json').read_text());v1z=np.load(v1/'states.npz');m=json.loads((p/'metrics.json').read_text());bm=json.loads((b/'metrics.json').read_text());z=np.load(p/'states.npz');bz=np.load(b/'states.npz');ref=bz['kinodynamic_reference_pose'];err=geometric_error(z['position_world_xy'],ref);berr=geometric_error(bz['position_world_xy'],ref)
  identical_ref=np.array_equal(ref,z['kinodynamic_reference_pose']);assert identical_ref
  initial_keys=['position_world_xy','yaw_world_rad','linear_velocity_world_m_s','angular_velocity_world_rad_s','joint_position_rad','joint_velocity_rad_s','applied_action','roll_world_rad']
  initial={k:bool(np.array_equal(z[k][0],bz[k][0])) for k in initial_keys};assert all(initial.values())
  fb=json.loads((h/('navigation_'+suite)/'feedback'/('bank_feedback_'+p.name[len('episode_'):]+'.json')).read_text());assert len(fb)==int(m['action_blocks'])*5
  common=min(len(z['applied_action']),len(bz['applied_action']));delta=abs(z['applied_action'][:common]-bz['applied_action'][:common]);changed=np.flatnonzero(np.any(delta>1e-6,axis=1))
  keys=['success','collision','timeout','runtime_error','elapsed_s','final_distance_m','path_length_m','minimum_clearance_m','termination_reason','spl']
  row=dict(suite=suite,episode=p.name,baseline={k:bm.get(k) for k in keys},candidate={k:m.get(k) for k in keys},baseline_physical_contacts=int(np.count_nonzero(bz['physical_contact'])),candidate_physical_contacts=int(np.count_nonzero(z['physical_contact'])),same_reference=identical_ref,same_initial_state=initial,baseline_geometric_cross_track_rms_m=float(np.sqrt(np.mean(berr**2))),candidate_geometric_cross_track_rms_m=float(np.sqrt(np.mean(err**2))),baseline_geometric_cross_track_peak_m=float(berr.max()),candidate_geometric_cross_track_peak_m=float(err.max()),first_different_applied_state_s=float(z['simulation_time_s'][changed[0]]) if len(changed) else None,pid_ticks=len(fb),pid_clipped_fraction=float(np.mean([r['pid_clipped'] for r in fb])),supervisor_fraction=float(np.mean([r['supervisor_active'] for r in fb])),bank_error_rms_deg=float(np.degrees(np.sqrt(np.mean([r['error_rad']**2 for r in fb])))))
  row['previous_pid']={k:v1m.get(k) for k in keys};row['previous_pid_cross_track_rms_m']=float(np.sqrt(np.mean(geometric_error(v1z['position_world_xy'],ref)**2)));mapping=json.loads((h/('navigation_'+suite)/'feedback'/('mapping_'+p.name[len('episode_'):]+'.json')).read_text());active=[r for r in mapping if r['active']];row['mapping_active_blocks']=len(active);row['mapping_curvature_clamp_fraction']=float(np.mean([r['curvature_clamped'] for r in active])) if active else 0;row['mapping_speed_clamp_fraction']=float(np.mean([r['speed_clamped'] for r in active])) if active else 0;rows.append(row)
  ax.plot(ref[:,0],ref[:,1],'k--',alpha=.5,label='fixed planned reference');ax.plot(bz['position_world_xy'][:,0],bz['position_world_xy'][:,1],label='B0');ax.plot(v1z['position_world_xy'][:,0],v1z['position_world_xy'][:,1],label='previous calibrated PID',color='orange',alpha=.7);ax.plot(z['position_world_xy'][:,0],z['position_world_xy'][:,1],label='tuned PID',color='green');ax.set_aspect('equal');ax.grid(True);ax.set_xlabel('world X [m]');ax.set_ylabel('world Y [m]');ylim=ax.get_ylim();center=sum(ylim)/2;span=max(ylim[1]-ylim[0],3.0);ax.set_ylim(center-span/2,center+span/2);ax.set_title(p.name.replace('episode_',''),fontsize=9);xlim=ax.get_xlim();center_x=sum(xlim)/2;span_x=max(xlim[1]-xlim[0],3.0);ax.set_xlim(center_x-span_x/2,center_x+span_x/2)
 for ax in axes[len(folders):]:ax.set_visible(False)
 handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='upper center',ncol=4,fontsize=9);fig.tight_layout(rect=[0,0,1,.95]);fig.savefig(h/f'{suite}_paired_paths.png',dpi=150);plt.close(fig)
(h/'navigation_comparison.json').write_text(json.dumps(rows,indent=2))
for r in rows:print(r['episode'],r['baseline']['success'],'->',r['candidate']['success'],r['candidate']['elapsed_s'],r['candidate']['final_distance_m'],'contacts',r['candidate_physical_contacts'])
