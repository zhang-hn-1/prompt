"""Independent posthoc assessment; no live policy imports or score rewriting."""
from pathlib import Path
import hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
H=Path(__file__).resolve().parent
W=H.parents[1]
OLD=H.parent/'visible_surface_connectivity_20261004_v1'
O=H.parent/'visual_information_new_rig_20261004_v2'

def read(path):return json.loads(path.read_text())
def projection(xy,path):
    start=path[:-1];delta=np.diff(path,axis=0);length=np.linalg.norm(delta,axis=1)
    valid=length>1e-12;start=start[valid];delta=delta[valid];length=length[valid]
    arc=np.r_[0,np.cumsum(length)]
    fraction=np.clip(np.sum((xy[:,None]-start)*delta,axis=2)/length**2,0,1)
    distance=np.linalg.norm(xy[:,None]-start-fraction[...,None]*delta,axis=2)
    index=np.argmin(distance,axis=1)
    return arc[index]+fraction[np.arange(len(xy)),index]*length[index],distance[np.arange(len(xy)),index]

def main():
    figures=H/'figures';figures.mkdir(exist_ok=True)
    output=dict(role='posthoc diagnostic audit, historical gates and scores unchanged',navigation={},observation={},provenance_limitations={})
    fig,axes=plt.subplots(1,2,figsize=(11,4.5))
    for ax,case in zip(axes,('e0','e2')):
        reference=np.load(O/f'common_reference_{case}.npz')['position_world_xy']
        ax.plot(reference[:,0],reference[:,1],':',color='gray',label='Frozen evaluation reference')
        roots=[('strict_old',OLD/'runs/surface'/case),('risk_old',H/'risk_runs'/case),('risk_flush',H/'risk_runs_corrected'/case)]
        if list((H/'risk_runs_kinematic_sync'/case).glob('episode_*/metrics.json')):roots.append(('risk_fk_sync',H/'risk_runs_kinematic_sync'/case))
        if list((H/'risk_runs_capture_transaction'/case).glob('episode_*/metrics.json')):roots.append(('risk_transaction',H/'risk_runs_capture_transaction'/case))
        output['navigation'][case]={}
        for name,root in roots:
            ep=next(root.glob('episode_*'));states=np.load(ep/'states.npz');actions=np.load(ep/'actions.npz');physics=np.load(root/'physics.npz');metrics=read(ep/'metrics.json')
            xy=states['position_world_xy'];progress,lateral=projection(xy,reference)
            records=read(root/'vision_records.json')
            manifest=read(ep/'manifest.json');goal=np.array(next(e for e in manifest['episodes'] if e['episode_id']==f'golden_{case}_0001')['goal_position_world_m'][:2])
            active=[i for i,r in enumerate(records) if np.any(r['request'])]
            result={k:metrics[k] for k in ('success','collision','termination_reason','elapsed_s','path_length_m','final_distance_m','goal_progress_m')}
            result.update(action_blocks=len(records),nonzero_action_blocks=len(active),first_nonzero_time_s=records[active[0]]['time_s'] if active else None,last_nonzero_time_s=records[active[-1]]['time_s'] if active else None,
                independent_reference_progress_final_m=float(progress[-1]-progress[0]),independent_reference_progress_peak_m=float(progress.max()-progress[0]),independent_cross_track_rms_m=float(np.sqrt(np.mean(lateral**2))),independent_cross_track_peak_m=float(lateral.max()),last_route_reason=records[-1]['route_reason'],source_identity_file=str(root/'experiment_identity.json'),physical_samples=len(physics['simulation_time_s']))
            if name!='strict_old':
                snapshot=np.load(root/'belief_snapshots/frame_000.npz');i,j=np.indices(snapshot['labels'].shape);xycell=snapshot['origin']+np.stack((i,j),axis=-1)*snapshot['resolution']
                center=xy[0];near=np.linalg.norm(xycell-center,axis=-1)<=.4+1e-5
                result['initial_footprint_cells']=int(near.sum());result['initial_footprint_free_cells']=int(((snapshot['labels']==0)&near).sum())
                clocks=read(root/'camera_clocks.json');result['captures']=len(clocks);result['max_camera_timestamp_age_s']=max(abs(v['age_s']) for c in clocks for v in c['eyes'].values())
            output['navigation'][case][name]=result
            if name not in ('risk_flush','risk_fk_sync'):ax.plot(xy[:,0],xy[:,1],label=name,alpha=.85)
        original=np.load(roots[1][1]/'physics.npz');corrected=np.load(roots[2][1]/'physics.npz');strict=np.load(roots[0][1]/'physics.npz')
        keys=('raw_command_target','smoothed_target','applied_action','root_position_world_m','root_quaternion_world_wxyz','joint_position_rad','joint_velocity_rad_s','simulation_time_s')
        output['navigation'][case]['old_flush_exact_identity']={k:bool(np.array_equal(original[k],corrected[k],equal_nan=True)) for k in keys}
        prefix=original['simulation_time_s']<=6.+1e-5
        output['navigation'][case]['strict_risk_prefix_physics_identity']={k:bool(np.array_equal(strict[k][:prefix.sum()],original[k][prefix],equal_nan=True)) for k in keys}
        if len(roots)>3:
            synced=np.load(roots[3][1]/'physics.npz')
            output['navigation'][case]['risk_fk_prefix_physics_identity']={k:bool(np.array_equal(original[k][prefix],synced[k][:prefix.sum()],equal_nan=True)) for k in keys}
        if len(roots)>4:
            transaction=np.load(roots[4][1]/'physics.npz')
            output['navigation'][case]['risk_transaction_prefix_physics_identity']={k:bool(np.array_equal(original[k][prefix],transaction[k][:prefix.sum()],equal_nan=True)) for k in keys}
        ax.scatter(*goal,marker='*',s=100,color='black',label='Goal');ax.set_title(case.upper()+': actual XY, original 35s budget');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(figures/'navigation_comparison.png',dpi=160);plt.close(fig)
    oldobs=read(H/'observation_assessment.json')
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    for name,r in oldobs.items():
        states=read(H/'observations'/name/'states.json');t=np.array([s['time_s'] for s in states]);pos=np.array([s['position'] for s in states]);v=np.array([s['velocity'] for s in states]);q=np.array([s['quaternion'] for s in states]);begin=int(np.flatnonzero(t>=6.-1e-5)[0]);delta=np.linalg.norm(np.diff(pos[:,:2],axis=0),axis=1)
        result={k:value for k,value in r.items() if k not in ('frames','stop_qualified_times_s')};result['path_after_settle_m']=float(delta[begin:].sum());result['analysis_version']='v2 excludes the 5.9->6.0 interval';result['acquisition']='historical timestamp-only; annotator queue limitation applies';output['observation'][name]=result
        bank=np.degrees(np.arcsin(np.clip(2*(q[:,0]*q[:,1]+q[:,2]*q[:,3]),-1,1)))
        axes[0].plot(pos[:,0]-pos[0,0],pos[:,1]-pos[0,1],label=name);axes[1].plot(t,np.linalg.norm(v[:,:2],axis=1),label=name);axes[2].plot(t,bank,label=name)
    for ax,title in zip(axes,('Actual translation','Measured XY speed','Measured body bank')):ax.set_title(title);ax.grid(alpha=.2);ax.legend(fontsize=8)
    axes[0].set_xlabel('X offset (m)');axes[0].set_ylabel('Y offset (m)');axes[0].axis('equal');axes[1].set_xlabel('Time (s)');axes[1].set_ylabel('m/s');axes[1].axhline(.05,color='gray',linestyle=':');axes[2].set_xlabel('Time (s)');axes[2].set_ylabel('deg')
    fig.tight_layout();fig.savefig(figures/'observation_motion.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for col,method in enumerate(('sgbm','fast')):
        before=read(H/('stereo_audit.json' if method=='sgbm' else 'fast_audit.json'));after=read(H/f'{method}_audit_corrected.json');seqs=list(after['groups']);x=np.arange(len(seqs))
        for row,metric in enumerate(('coverage','within_0p10m')):
            a=axes[row,col]
            for offset,data,label in ((-.18,before,'Original capture'),(.18,after,'Fixed render barrier')):
                vals=[data['groups'][s]['body_overlap'][metric] for s in seqs];a.bar(x+offset,[np.nan if v is None else v for v in vals],width=.35,label=label)
            a.set_xticks(x);a.set_xticklabels([s.replace('hard_0_e1_000','h').replace('direct_e1','direct') for s in seqs],rotation=20);a.set_ylim(0,1);a.set_ylabel(metric);a.set_title(method+' body-height binocular-overlap subset');a.grid(axis='y',alpha=.2);a.legend(fontsize=7)
            if row==1:a.axhline(.9,linestyle=':',color='black',label='Frozen gate')
    fig.tight_layout();fig.savefig(figures/'stereo_accuracy_coverage.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(10,7))
    for col,index in enumerate((0,3)):
        z=np.load(H/'static_geometry'/f'board_{index}.npz');axes[0,col].imshow(z['left_rgb']);axes[0,col].set_title('Same physics time: render '+str(index));axes[1,col].imshow(z['left_depth'],vmin=0,vmax=4,cmap='viridis');axes[1,col].set_title('Depth, fixed scale 0-4m')
    for a in axes.ravel():a.axis('off')
    fig.tight_layout();fig.savefig(figures/'static_queued_geometry.png',dpi=150);plt.close(fig)
    output['provenance_limitations'].update(risk_unknown_labels_unchanged_claim=False,reason='Inherited seed_body writes unknown within rolling .4m footprint to free; map has no separate prior source label.',spatial_memory_is_separate_and_not_used_by_live_risk_policy=True,footprint_protocol_mismatch_is_not_retroactively_fixed=True,observation_visibility_uses_old_acquisition=True,estimated_pose_navigation_run=False,learned_world_model_run=False,formal_new_success_rate_reported=False)
    output['source_sha256']={str(Path(__file__)):hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (H/'stage_assessment_v2.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output['navigation'],indent=2));print('STAGE_ASSESSMENT_COMPLETE')
if __name__=='__main__':main()
