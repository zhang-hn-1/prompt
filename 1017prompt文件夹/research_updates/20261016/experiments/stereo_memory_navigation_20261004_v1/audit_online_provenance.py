"""Independent offline scoring: live memory evidence plus frozen occupancy proxy.

Occupancy is a static2D footprint label, not per-point3D collision ground truth.
No evaluation label is ever returned to the live policy. Preserve old results.
"""
from pathlib import Path
import json,hashlib,subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from summarize_stage import projection
H=Path(__file__).resolve().parent

def read(p):return json.loads(p.read_text())

def main():
    protocol=read(H/'online_provenance_protocol.json')
    reference_file=H/'online_provenance_evaluation_reference_e1.npz'
    reference=np.load(reference_file)['position_world_xy']
    output=dict(role='actual privileged-depth causal memory diagnostic, not admitted RGB/formal success rate',arms={},memory_proxy_scope='Frozen2D occupancy overlap and previously observed2D hazard persistence; cannot certify full3D swept volume',reference_file=str(reference_file),reference_sha256=hashlib.sha256(reference_file.read_bytes()).hexdigest())
    fig,axes=plt.subplots(1,3,figsize=(15,4.5),constrained_layout=True)
    axes[0].plot(reference[:,0],reference[:,1],':',color='gray',label='Frozen evaluation reference')
    for mode in protocol['arms']:
        root=H/'online_provenance_runs'/mode;ep=root/'episode_golden_e1_0001'
        metrics=read(ep/'metrics.json');state=np.load(ep/'states.npz');phys=np.load(root/'physics.npz');records=read(root/'vision_records.json')
        identity=read(root/'experiment_identity.json')
        assert identity['actual_policy']=='provenance_policy.InformationPolicy' and identity['memory_mode']==mode and not identity['body_prior_observed_free']
        assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==sha for p,sha in identity['source_sha256'].items())
        xy=state['position_world_xy'];progress,lateral=projection(xy,reference)
        active=[r for r in records if np.any(r['request'])]
        contacts=[s for s in read(ep/'collision_audit.json')['samples'] if s.get('contact')]
        occ=np.load(root/'offline_occupancy_label.npz');initial=np.load(root/'belief_snapshots/frame_000.npz')
        i,j=np.indices(initial['labels'].shape);world=initial['origin']+np.stack((i,j),axis=-1)*initial['resolution']
        tcells=np.rint((world-occ['origin'])/occ['resolution']).astype(int)
        valid=(tcells>=0).all(-1)&(tcells<np.array(occ['labels'].shape)).all(-1)
        teacher=np.full(initial['labels'].shape,-1,np.int8);teacher[valid]=occ['labels'][tcells[...,0][valid],tcells[...,1][valid]]
        ever_valid_hazard=np.zeros(initial['labels'].shape,bool);snaps={int(f.stem.split('_')[1]):f for f in (root/'belief_snapshots').glob('frame_*.npz')}
        map_rows=[]
        for f in sorted((root/'privileged_sensor_packets').glob('frame_*.npz')):
            index=int(f.stem.split('_')[1]);packet=np.load(f);points=packet['points_world'];points=points[np.isfinite(points).all(1)&(np.linalg.norm(points-packet['root_xyz'],axis=1)>=.55)]
            hazard=points[(points[:,2]>.05)&(points[:,2]<=1.5)]
            cells=np.rint((hazard[:,:2]-initial['origin'])/initial['resolution']).astype(int)
            inside=(cells>=0).all(1)&(cells<np.array(teacher.shape)).all(1);cells=cells[inside]
            current_confirmed=np.zeros(teacher.shape,bool)
            if len(cells):
                confirmed=teacher[cells[:,0],cells[:,1]]==1;cells=cells[confirmed];current_confirmed[cells[:,0],cells[:,1]]=True
            ever_valid_hazard|=current_confirmed
            if index not in snaps:continue
            s=np.load(snaps[index]);labels=s['labels'];codes=s['provenance']
            assert np.array_equal(labels==-1,codes==0)
            assert np.array_equal(labels==0,np.isin(codes,[1,2]))
            assert np.array_equal(labels==1,np.isin(codes,[3,4]))
            assert np.all(np.isfinite(s['floor_time'][labels==0]))
            assert np.all(np.isfinite(s['hazard_time'][labels==1]))
            assert np.all(s['floor_time'][np.isfinite(s['floor_time'])]<=s['time_s']+1e-6)
            assert np.all(s['hazard_time'][np.isfinite(s['hazard_time'])]<=s['time_s']+1e-6)
            prior_hidden=ever_valid_hazard&~current_confirmed
            floor=(labels==0)&valid
            map_rows.append(dict(time_s=float(s['time_s']),known_floor_valid_n=int(floor.sum()),floor_overlap_occupied_proxy_n=int((floor&(teacher==1)).sum()),known_hazard_valid_n=int(((labels==1)&valid).sum()),hazard_overlap_occupied_proxy_n=int(((labels==1)&(teacher==1)).sum()),ever_observed_confirmed_hazard_n=int(ever_valid_hazard.sum()),currently_hidden_past_confirmed_hazard_n=int(prior_hidden.sum()),retained_hidden_confirmed_hazard_n=int((prior_hidden&(labels==1)).sum()),forgotten_hidden_confirmed_hazard_n=int((prior_hidden&(labels!=1)).sum()),current_floor=int((codes==1).sum()),history_floor=int((codes==2).sum()),history_occupied=int((codes==4).sum()),footprint_unknown=int((s['footprint_geometry_mask']&(labels==-1)).sum())))
        clocks=read(root/'camera_clocks.json');speed=np.linalg.norm(phys['linear_velocity_world_m_s'][:,:2],axis=1)
        peak_free_proxy=max((r['floor_overlap_occupied_proxy_n'] for r in map_rows),default=0)
        result={k:metrics[k] for k in ('success','collision','termination_reason','elapsed_s','path_length_m','final_distance_m','goal_progress_m','minimum_clearance_m')}
        result.update(policy_calls=len(records),captures=len(clocks),physical_samples=len(phys['simulation_time_s']),nonzero_blocks=len(active),first_nonzero_s=active[0]['time_s'] if active else None,last_nonzero_s=active[-1]['time_s'] if active else None,independent_reference_progress_final_m=float(progress[-1]-progress[0]),independent_reference_progress_peak_m=float(progress.max()-progress[0]),cross_track_rms_m=float(np.sqrt(np.mean(lateral**2))),cross_track_peak_m=float(lateral.max()),peak_actual_xy_speed_m_s=float(speed.max()),initial_observed_floor_n=int((initial['labels']==0).sum()),initial_footprint_observed_free_n=int(((initial['labels']==0)&initial['footprint_geometry_mask']).sum()),body_prior_observed_free=0,max_camera_timestamp_age_s=max(abs(v['age_s']) for c in clocks for v in c['eyes'].values()),last_route_reason=records[-1]['route_reason'],first_scored_contact=contacts[0] if contacts else None,peak_floor_occupied_overlap_proxy_n=peak_free_proxy,peak_retained_hidden_confirmed_hazard_n=max(r['retained_hidden_confirmed_hazard_n'] for r in map_rows),peak_forgotten_hidden_confirmed_hazard_n=max(r['forgotten_hidden_confirmed_hazard_n'] for r in map_rows),map_rows=map_rows,source_identity_verified=True)
        result['video_metadata']={}
        for name in ('third_person','stereo_view'):
            data=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=nb_frames,r_frame_rate,duration','-of','json',str(ep/(name+'.mp4'))],text=True))['streams'][0]
            result['video_metadata'][name]=data
        output['arms'][mode]=result
        axes[0].plot(xy[:,0],xy[:,1],label=mode);axes[1].plot([r['time_s'] for r in map_rows],[r['retained_hidden_confirmed_hazard_n'] for r in map_rows],label=mode);axes[2].plot(records and [r['time_s'] for r in records],[r['known_occupied'] for r in records],label=mode)
    a=np.load(H/'online_provenance_runs/current/physics.npz');b=np.load(H/'online_provenance_runs/history/physics.npz');prefix=a['simulation_time_s']<=6.+1e-5
    output['same_6s_prefix_fields']={k:bool(np.array_equal(a[k][prefix],b[k][:int(prefix.sum())],equal_nan=True)) for k in a.files if k not in ('raw_simulation_time_s','raw_physics_step_index')}
    assert all(output['same_6s_prefix_fields'].values())
    output['raw_initial_clock_delta_history_minus_current_s']=float(b['raw_simulation_time_s'][0]-a['raw_simulation_time_s'][0])
    for arm in output['arms'].values():assert arm['initial_footprint_observed_free_n']==0
    output['interface_verified']=True
    output['navigation_success']=False
    output['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),H/'online_provenance_protocol.json',H/'provenance_grid.py',H/'provenance_policy.py',H/'spatial_memory.py',H/'summarize_stage.py']}
    for ax,title in zip(axes,('Actual XY on frozen reference','Retained hidden occupied proxy','Actual online occupied labels')):
        ax.set_title(title);ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes[0].axis('equal');axes[0].set_xlabel('world X (m)');axes[0].set_ylabel('world Y (m)');axes[1].set_xlabel('time (s)');axes[2].set_xlabel('time (s)')
    fig.savefig(H/'figures/online_provenance_comparison.png',dpi=150);plt.close(fig)
    (H/'online_provenance_assessment.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({mode:{k:v for k,v in r.items() if k not in ('map_rows','video_metadata')} for mode,r in output['arms'].items()},indent=2));print('ONLINE_PROVENANCE_AUDIT_COMPLETE')
if __name__=='__main__':main()
