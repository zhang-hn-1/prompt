"""Independent same-start control comparison, causal filter replay and prefix audit."""
import hashlib,json,subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from provenance_grid import ProvenanceGrid
from stopping_consumer import StoppingConsumer
from summarize_stage import projection
H=Path(__file__).resolve().parent
read=lambda p:json.loads(p.read_text())

def assess(root):
    ep=root/'episode_golden_e1_0001';metrics=read(ep/'metrics.json');state=np.load(ep/'states.npz');p=np.load(root/'physics.npz');r=read(root/'vision_records.json');
    np.testing.assert_array_equal(np.asarray([v['request'] for v in r],np.float32),np.load(ep/'actions.npz')['action_blocks'])
    identity=read(root/'experiment_identity.json')
    assert all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==sha for f,sha in identity['source_sha256'].items())
    reference=np.load(H/'online_provenance_evaluation_reference_e1.npz')['position_world_xy']
    progress,lateral=projection(state['position_world_xy'],reference)
    contacts=[v for v in read(ep/'collision_audit.json')['samples'] if v.get('contact')]
    active=[v for v in r if np.any(v['request'])]
    out={k:metrics[k] for k in ('success','collision','termination_reason','elapsed_s','path_length_m','final_distance_m','goal_progress_m')}
    out.update(policy_calls=len(r),physical_samples=len(p['simulation_time_s']),nonzero_blocks=len(active),last_nonzero_s=active[-1]['time_s'] if active else None,
        independent_reference_progress_final_m=float(progress[-1]-progress[0]),independent_reference_progress_peak_m=float(progress.max()-progress[0]),
        cross_track_rms_m=float(np.sqrt(np.mean(lateral**2))),cross_track_peak_m=float(lateral.max()),
        peak_actual_xy_speed_m_s=float(np.linalg.norm(p['linear_velocity_world_m_s'][:,:2],axis=1).max()),
        first_scored_contact=contacts[0] if contacts else None,source_identity_verified=True)
    out['videos']={name:json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=nb_frames,r_frame_rate,duration','-of','json',str(ep/(name+'.mp4'))],text=True))['streams'][0] for name in ('third_person','stereo_view')}
    return out,state,p,r


def main():
    baseline,sa,pa,ra=assess(H/'online_provenance_runs/history')
    candidate,sb,pb,rb=assess(H/'local_stopping_run')
    identity=read(H/'local_stopping_run/experiment_identity.json');assert identity['actual_policy']=='stopping_policy.InformationPolicy'
    bident=read(H/'online_provenance_runs/history/experiment_identity.json')
    shared={f:sha for f,sha in bident['source_sha256'].items() if f in identity['source_sha256']}
    assert all(identity['source_sha256'][f]==sha for f,sha in shared.items())
    initial=np.load(H/'local_stopping_run/belief_snapshots/frame_000.npz');grid=ProvenanceGrid(initial['origin']);consumer=StoppingConsumer()
    snapshot_indices=[]
    for i,r in enumerate(rb):
        packet=np.load(H/'local_stopping_run/privileged_sensor_packets'/f'frame_{i:03d}.npz')
        grid.update_points(packet['points_world'],packet['root_xyz'],float(packet['time_s']));grid.seed_body(packet['root_xyz'][:2])
        final,info=consumer.apply(r['proposed_request'],r['position_world_xy'],r['speed_xy'],grid.labels,grid.origin,grid.resolution)
        np.testing.assert_array_equal(final,r['request']);assert info==r['stopping_consumer']
        snap=H/'local_stopping_run/belief_snapshots'/f'frame_{i:03d}.npz'
        if snap.exists():
            z=np.load(snap)
            for a,b in [(grid.labels,z['labels']),(grid.memory.floor_time,z['floor_time']),(grid.memory.hazard_time,z['hazard_time']),(grid.provenance(),z['provenance'])]:np.testing.assert_array_equal(a,b)
            snapshot_indices.append(i)
    vetoes=[r for r in rb if r['stopping_consumer']['stop_veto']]
    assert vetoes,'Candidate did not change requests'
    first=vetoes[0];t=first['time_s'];prefix=pb['simulation_time_s']<=t+1e-5;n=int(prefix.sum())
    common=[k for k in pa.files if k not in ('raw_simulation_time_s','raw_physics_step_index')]
    equal={k:bool(np.array_equal(pa[k][:n],pb[k][:n],equal_nan=True)) for k in common};assert all(equal.values())
    before=[i for i,r in enumerate(rb) if r['time_s']<t-1e-5]
    assert all(ra[i]=={k:v for k,v in rb[i].items() if k not in ('stopping_consumer','proposed_request')} for i in before)
    j=np.flatnonzero(pb['simulation_time_s']>=t-1e-5)[0];start=pb['root_position_world_m'][j,:2]
    end=pb['root_position_world_m'][-1,:2]
    after=pb['root_position_world_m'][j:,:2]
    count=0;qualified=None
    for k in np.flatnonzero((pb['physics_step_in_block']==5)&(pb['simulation_time_s']>=t-1e-5)):
        speed=float(np.linalg.norm(pb['linear_velocity_world_m_s'][k,:2]));count=count+1 if speed<.05 else 0
        if count>=5:qualified=float(pb['simulation_time_s'][k]);break
    q=pb['root_quaternion_world_wxyz'];bank=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),-1,1)))
    zero_indices=np.flatnonzero((pb['simulation_time_s']>t+1e-5)&(abs(pb['smoothed_target'][:,0])<1e-6))
    qual_index=int(np.searchsorted(pb['simulation_time_s'],qualified-1e-5)) if qualified is not None else len(after)-1+j
    horizon_index=min(len(pb['simulation_time_s'])-1,int(np.searchsorted(pb['simulation_time_s'],t+1.5-1e-5)))
    candidate.update(bank_at_first_veto_deg=float(bank[j]),
        path_at_first_veto_plus1p5s_m=float(np.linalg.norm(np.diff(pb['root_position_world_m'][j:horizon_index+1,:2],axis=0),axis=1).sum()),
        radial_at_first_veto_plus1p5s_m=float(np.linalg.norm(pb['root_position_world_m'][horizon_index,:2]-start)),
        path_until_first_stop_qualification_m=float(np.linalg.norm(np.diff(pb['root_position_world_m'][j:qual_index+1,:2],axis=0),axis=1).sum()),
        radial_at_first_stop_qualification_m=float(np.linalg.norm(pb['root_position_world_m'][qual_index,:2]-start)),
        actual_bank_range_deg=[float(bank.min()),float(bank.max())],
        empirical_observation_abs_bank_peak_deg=14.041426773958973,physical_samples_beyond_empirical_bank=int((abs(bank)>14.041426773958973).sum()),
        first_smoothed_main_zero_after_veto_s=float(pb['simulation_time_s'][zero_indices[0]]) if len(zero_indices) else None,
        veto_blocks=len(vetoes),first_veto_s=t,first_veto_speed_m_s=first['speed_xy'],
        actual_five_sample_stop_after_veto_s=qualified,post_first_veto_path_to_end_m=float(np.linalg.norm(np.diff(after,axis=0),axis=1).sum()),
        post_first_veto_radial_peak_m=float(np.linalg.norm(after-start,axis=1).max()),post_first_veto_net_to_end_m=float(np.linalg.norm(end-start)),
        outside_empirical_speed_policy_calls=sum(r['stopping_consumer']['beyond_empirical_speed'] for r in rb),
        last_route_reason=rb[-1]['route_reason'],last_action_class=rb[-1]['action_class'])
    # Check capture history insertion exactly once, independently of act/filter replay.
    insertion_counts={}
    for key in ('proprio_history','action_history','quaternion_world_wxyz_history'):
        count=0;prior=None
        for f in sorted((H/'local_stopping_run/online_inputs').glob('frame_*.npz')):
            current=np.load(f)[key]
            if prior is not None:np.testing.assert_array_equal(current[:-1],prior[1:]);count+=1
            prior=current
        insertion_counts[key]=count
    ref=H/'online_provenance_evaluation_reference_e1.npz'
    result=dict(role='one actual local stopping-consumer experiment, privileged depth/pose; no RGB or formal E0/E1/E2 result',
        baseline=baseline,candidate=candidate,first_veto_record=first,
        pre_divergence_physical_fields_equal=equal,pre_divergence_record_count=len(before),
        raw_initial_clock_delta_s=float(pb['raw_simulation_time_s'][0]-pa['raw_simulation_time_s'][0]),
        shared_frozen_source_count=len(shared),all_candidate_filter_requests_replayed=True,all_saved_memory_snapshots_replayed=snapshot_indices,
        causal_history_insertions_verified=insertion_counts,reference_sha256=hashlib.sha256(ref.read_bytes()).hexdigest(),
        navigation_accepted=bool(candidate['success'] and not candidate['collision']),
        empirical_stop_envelope_certified=False,
        source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (Path(__file__),H/'local_stopping_protocol.json',H/'local_execution_gap.json')})
    (H/'local_stopping_assessment.json').write_text(json.dumps(result,indent=2)+'\n')
    fig,axes=plt.subplots(1,3,figsize=(15,4.5),constrained_layout=True);refxy=np.load(ref)['position_world_xy'];axes[0].plot(refxy[:,0],refxy[:,1],':',color='gray',label='Frozen reference')
    for label,s,p,r in [('History B0',sa,pa,ra),('Stopping veto',sb,pb,rb)]:
        xy=s['position_world_xy'];prog,lat=projection(xy,refxy)
        axes[0].plot(xy[:,0],xy[:,1],label=label);axes[1].plot(s['simulation_time_s'],prog-prog[0],label=label);axes[2].plot(p['simulation_time_s'],np.linalg.norm(p['linear_velocity_world_m_s'][:,:2],axis=1),label=label)
    axes[0].plot(start[0],start[1],'ro',label='First veto');axes[0].axis('equal');axes[2].axhline(.05,color='gray',ls=':',label='Stop qualification speed')
    for ax,title in zip(axes,('Actual XY','Independent reference progress','Actual world XY speed')):ax.set_title(title);ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes[0].set_xlabel('world X(m)');axes[0].set_ylabel('world Y(m)');axes[1].set_xlabel('time(s)');axes[2].set_xlabel('time(s)')
    fig.savefig(H/'figures/local_stopping_comparison.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','all_saved_memory_snapshots_replayed','pre_divergence_physical_fields_equal','first_veto_record')},indent=2))
if __name__=='__main__':main()
