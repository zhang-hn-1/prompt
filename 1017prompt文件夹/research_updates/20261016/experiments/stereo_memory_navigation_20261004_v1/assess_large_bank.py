"""Pre-frozen physical definitions on two whole targeted sequences; no model fit."""
import hashlib,json,subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from motion_measurement import xy_curvature,spatial_curvature,qualification
H=Path(__file__).resolve().parent
read=lambda f:json.loads(f.read_text())

def main():
    result=dict(role='two new targeted development turn-stop measurements; no navigation/fresh model validation',arms={})
    fig,axs=plt.subplots(2,3,figsize=(14,8),constrained_layout=True)
    for name in ('positive','negative'):
        r=H/'large_bank_runs'/name;identity=read(r/'identity.json');assert identity['complete']
        assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity['source_sha256'].items())
        p=np.load(r/'physics.npz');states=read(r/'states.json');t=np.array([s['time_s'] for s in states]);xy=np.array([s['position'][:2] for s in states]);vel=np.array([s['velocity'][:2] for s in states]);q=np.array([s['quaternion'] for s in states]);q/=np.linalg.norm(q,axis=1)[:,None]
        bank=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),-1,1)))
        yaw=np.unwrap(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2)))
        speed=np.linalg.norm(vel,axis=1);k,course,valid=xy_curvature(t,xy,vel);kp=spatial_curvature(xy)
        np.testing.assert_array_equal(xy,p['position_world_xy'][::5])
        assert len(t)==181 and len(p['simulation_time_s'])==901
        active=(t>=6-1e-5)&(t<=12+1e-5);tail=(t>=10.5-1e-5)&(t<=12+1e-5)
        mainvel=p['joint_velocity_rad_s'][::5,0];slope=float(np.polyfit(t[tail],bank[tail],1)[0]);cv=float(np.std(speed[tail])/np.mean(speed[tail]))
        stable=bool(bank[tail].std()<=2 and abs(slope)<=1 and cv<=.2 and mainvel[tail].std()<=.03)
        desired=22.5 if name=='positive' else -22.5;count=0;reached=None
        for i in np.flatnonzero(active):
            count=count+1 if abs(bank[i]-desired)<=2 else 0
            if count>=5 and reached is None:reached=float(t[i])
        stop_indices=np.flatnonzero(t>=12-1e-5);qualified,counts=qualification(speed[stop_indices]);first=np.flatnonzero(qualified)
        qualified_time=float(t[stop_indices[first[0]]]) if len(first) else None
        pt=p['simulation_time_s'];pxy=p['position_world_xy'];s=int(np.searchsorted(pt,12-1e-5));start=pxy[s];stops={}
        for label,end in [('1p5',13.5),('3',15.),('end',18.)]+([('qualified',qualified_time)] if qualified_time is not None else []):
            z=int(np.searchsorted(pt,end-1e-5));seg=pxy[s:z+1]
            stops[label]=dict(time_s=float(pt[z]),path_m=float(np.linalg.norm(np.diff(seg,axis=0),axis=1).sum()),net_m=float(np.linalg.norm(seg[-1]-start)),radial_peak_m=float(np.linalg.norm(seg-start,axis=1).max()))
        j=stop_indices[0];zero=np.flatnonzero((pt>12+1e-5)&(abs(p['smoothed_target'][:,0])<1e-6));look=[1.5,3,6];start_active=np.flatnonzero(t>=6-1e-5)[0];transitions=[]
        for horizon in look:
            z=int(np.searchsorted(t,6+horizon-1e-5));d=xy[z]-xy[start_active];c,ss=np.cos(yaw[start_active]),np.sin(yaw[start_active]);dlocal=np.array([[c,ss],[-ss,c]])@d
            transitions.append(dict(horizon_s=horizon,displacement_initial_yaw_frame_m=dlocal.tolist(),terminal_speed_m_s=float(speed[z]),terminal_bank_deg=float(bank[z]),delta_root_yaw_rad=float(yaw[z]-yaw[start_active])))
        identity_history={}
        for key in ('proprio_history','action_history','quaternion_world_wxyz_history'):
            prior=None;n=0
            for f in sorted((r/'history').glob('*.npz')):
                current=np.load(f)[key]
                if prior is not None:np.testing.assert_array_equal(current[:-1],prior[1:]);n+=1
                prior=current
            identity_history[key]=n
        stats=lambda v:dict(n=int(np.isfinite(v).sum()),median=float(np.nanmedian(v)) if np.isfinite(v).any() else None,p10=float(np.nanquantile(v,.1)) if np.isfinite(v).any() else None,p90=float(np.nanquantile(v,.9)) if np.isfinite(v).any() else None)
        wz=p['angular_velocity_world_rad_s'][::5,2];signed=vel[:,0]*np.cos(yaw)+vel[:,1]*np.sin(yaw)
        yaw_rate=np.gradient(yaw,t);yaw_k=np.where(abs(signed)>=.05,yaw_rate/np.where(abs(signed)>=.05,signed,np.nan),np.nan)
        row=dict(duration_s=float(t[-1]),captures=len(t),physics_samples=len(pt),raw_request_blocks=180,archived_pairs=37,
            physical_path_active6to12_m=float(np.linalg.norm(np.diff(xy[start_active:j+1],axis=0),axis=1).sum()),
            actual_speed_peak_m_s=float(speed.max()),actual_bank_range_deg=[float(bank.min()),float(bank.max())],target_bank_reached5times_s=reached,
            measured_steady_tail=stable,bank_tail_mean_deg=float(bank[tail].mean()),bank_tail_std_deg=float(bank[tail].std()),bank_tail_slope_deg_s=slope,
            speed_tail_mean_m_s=float(speed[tail].mean()),speed_tail_cv=cv,main_actual_tail_std_rad_s=float(mainvel[tail].std()),
            velocity_course_curvature_tail=stats(k[tail]),three_position_curvature_tail=stats(kp[tail]),
            root_yaw_over_signed_forward_speed_curvature_tail=stats(yaw_k[tail]),world_z_angular_velocity_tail_rad_s=stats(wz[tail]),
            actual_speed_at_stop_request_m_s=float(speed[j]),actual_bank_at_stop_request_deg=float(bank[j]),
            first_main_smoothed_zero_after12_s=float(pt[zero[0]]) if len(zero) else None,
            first_five_10hz_actual_stop_s=qualified_time,stopping=stops,finite_transitions=transitions,
            history_insertions_once=identity_history,source_identity_verified=True)
        row['video_metadata']={n:json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=nb_frames,r_frame_rate,duration','-of','json',str(r/(n+'.mp4'))],text=True))['streams'][0] for n in ('third_person','stereo_view')}
        result['arms'][name]=row
        axs[0,0].plot(xy[:,0],xy[:,1],label=name);axs[0,1].plot(t,bank,label=name);axs[0,2].plot(t,speed,label=name)
        axs[1,0].plot(t,k,label=name+' XYcourse');axs[1,0].plot(t,kp,':',label=name+' XYposition')
        radial=np.linalg.norm(xy-xy[j],axis=1);axs[1,1].plot(t[stop_indices]-12,radial[stop_indices],label=name)
        axs[1,2].plot(t,np.degrees(course-yaw),label=name)
        np.savez_compressed(r/'aligned_measurements.npz',time_s=t,xy=xy,speed_xy=speed,bank_deg=bank,root_yaw_rad=yaw,course_rad=course,course_curvature_1_m=k,position_curvature_1_m=kp,root_yaw_over_signed_speed_curvature_1_m=yaw_k,world_z_angular_velocity_rad_s=wz)
    a=np.load(H/'large_bank_runs/positive/physics.npz');b=np.load(H/'large_bank_runs/negative/physics.npz');mask=a['simulation_time_s']<=6+1e-5;n=int(mask.sum())
    result['identical_6s_prefix_nonraw']={k:bool(np.array_equal(a[k][:n],b[k][:n],equal_nan=True)) for k in a.files if k not in ('raw_simulation_time_s','raw_physics_step_index')};assert all(result['identical_6s_prefix_nonraw'].values())
    result['raw_origin_negative_minus_positive_s']=float(b['raw_simulation_time_s'][0]-a['raw_simulation_time_s'][0])
    result['frozen_old_stop_tail_m']=.14088407559444638;result['universal_bound_accepted']=False;result['navigation_tested']=False
    result['source_sha256']={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(__file__),H/'motion_measurement.py',H/'large_bank_protocol.json')}
    for ax,title in zip(axs.flat,('Actual open-plane XY','Measured bank','Actual world XY speed','Geometric XY curvature','Stop radial displacement','Course minus root heading (deg)')):
        ax.set_title(title);ax.grid(alpha=.2);ax.legend(fontsize=7)
    axs[0,0].axis('equal');axs[1,1].axhline(.14088407559444638,color='gray',ls=':',label='Old empirical tail')
    axs[1,0].set_ylim(-5,5);axs[1,0].set_ylabel('1/m; display cropped only, statistics use all valid samples')
    fig.savefig(H/'figures/large_bank_turn_stop.png',dpi=150);plt.close(fig)
    (H/'large_bank_assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
