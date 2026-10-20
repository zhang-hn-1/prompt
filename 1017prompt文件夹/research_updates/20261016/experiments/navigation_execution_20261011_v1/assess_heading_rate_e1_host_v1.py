"""Measured navigation comparison and current-input correction reconstruction."""
from pathlib import Path
import sys,hashlib,json,math
import numpy as np
import cv2

HERE=Path(__file__).resolve().parent
LEGACY=HERE.parent/'stereo_memory_navigation_20261004_v1'
sys.path.insert(0,str(LEGACY))
sys.path.insert(0,str(LEGACY.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from baseline35_metrics_v1 import assess,observation_gap_diagnostics
from assess_sgbm_risk_lifetime_v1 import complete_execution_evidence,cross_metrics
from assess_dense_current_live_e1_v2 import source_replay
from dense_current_live_termination_v1 import validate_termination
from heading_rate import projected_heading_rate
from heading_rate_metrics import navigation_gates,feedback_corrections

sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify(table):
    for p,d in table.items():
        if sha(p)!=d:
            raise ValueError('identity mismatch: '+str(p))

def evaluate_arm(name):
    run=HERE/f'heading_rate_e1_{name}_run_v2'
    cfg=json.loads((HERE/f'heading_rate_e1_{name}_protocol_v3.json').read_text())
    verify(cfg['source_sha256']);verify(cfg['input_sha256'])
    status=json.loads((HERE/f'heading_rate_e1_{name}_status_v2.json').read_text())
    assert status['status']=='finished' and status['child_returncode']==0 and not status['timeout'] and status['parent_signal'] is None
    manifest=json.loads((run/'artifact_manifest_post_close.json').read_text())
    assert manifest['status']==status;verify(manifest['artifact_sha256'])
    identity=json.loads((run/'identity.json').read_text())
    assert identity['complete'] and identity['reset_count']==1 and not identity['internal_reset']
    assert identity['heading_rate_feedback_enabled']==(name=='candidate') and identity['live_packet_sampler']=='original_stride2'
    assert identity['source_sha256']==cfg['source_sha256'] and identity['input_sha256']==cfg['input_sha256']
    states=json.loads((run/'states.json').read_text());contacts=json.loads((run/'physics_contacts.json').read_text())
    physics=dict(np.load(run/'physics.npz'));requests=np.load(run/'requests.npz')['requests'];rows=json.loads((run/'policy_records.json').read_text())
    assert len(rows)==len(requests)==identity['actions'] and len(states)==identity['captures']
    np.testing.assert_array_equal(requests,np.asarray([r['request'] for r in rows],np.float32))
    np.testing.assert_array_equal(physics['raw_command_target'][1:],requests.reshape(-1,2))
    normalized=complete_execution_evidence(states,contacts,physics['simulation_time_s'],len(requests))
    episode=json.loads((run/'task_episode.json').read_text())['actual_episode']
    start=np.array(episode['start_position_world_m'][:2]);goal=np.array(episode['goal_position_world_m'][:2])
    metrics=assess(states,normalized,start,goal);cross=cross_metrics(states,start,goal,metrics['scored_end_s'])
    metrics.update(arrival=metrics['success'],net_goal_axis_progress_m=metrics['along_start_goal_axis_m'],
        cross_rms_m=cross['rms_m'],cross_peak_m=cross['peak_m'],raw_contacts_zero=all(not c['contact'] for c in contacts))
    metrics['execution_stop_class']=validate_termination(json.loads((run/'termination.json').read_text()),metrics,identity,len(requests))
    reconstruction=[]
    for row in rows:
        i=row['frame'];diag=row.get('heading_rate_tracking')
        assert row['known_GT_pose'] and not row['native_pose_used'] and row.get('measured_free',0)==0
        if diag and diag.get('valid',False) and diag['source']=='tracking':
            current=i*5
            rate=projected_heading_rate(physics['root_quaternion_world_wxyz'][current],physics['angular_velocity_world_rad_s'][current])
            assert rate['supported']==diag['heading_rate_geometry_supported']
            if rate['supported']:
                np.testing.assert_allclose(diag['measured_projected_heading_rate_rad_s'],rate['yaw_rate_rad_s'],atol=1e-6,rtol=0)
            else:
                assert diag['measured_projected_heading_rate_rad_s'] is None
                assert not diag['heading_rate_consumer_applied']
            np.testing.assert_allclose(diag['measured_world_yaw_rate_rad_s'],physics['angular_velocity_world_rad_s'][current,2],atol=1e-6,rtol=0)
            speed=float(np.linalg.norm(physics['linear_velocity_world_m_s'][current,:2]))
            np.testing.assert_allclose(speed,diag['world_xy_speed_m_s'],atol=1e-6,rtol=0)
            sign=1 if diag['main_inverse_query_rad_s']>0 else -1
            desired=diag['requested_curvature_1_m']*sign*max(speed,.2)
            np.testing.assert_allclose(desired,diag['desired_world_yaw_rate_rad_s'],atol=1e-6,rtol=0)
            # The frozen inherited Oracle default gain is bound by its source identity.
            from benchmark.policies import DEFAULT_YAW_RATE_FEEDBACK_GAIN_S
            old,new=feedback_corrections(rate,desired=desired,sign=sign,gain=DEFAULT_YAW_RATE_FEEDBACK_GAIN_S)
            np.testing.assert_allclose(diag['heading_rate_feedback_before_rad'],old,atol=1e-6,rtol=0)
            used=new if name=='candidate' else old
            np.testing.assert_allclose(diag['yaw_rate_feedback_auxiliary_rad'],used,atol=1e-6,rtol=0)
            mapping=diag['calibrated_bank_mapping'];assert mapping['active']
            from bank_mapping import BankMap
            bank=BankMap(json.loads((LEGACY.parent/'oracle_feasible_route_20260929_v9/hybrid/bank_mapping.json').read_text()))
            independently_mapped=bank.query(diag['main_inverse_query_rad_s'],speed,diag['requested_curvature_1_m'])
            np.testing.assert_allclose(mapping['encoded_aux_rad'],independently_mapped['encoded_aux_rad'],atol=1e-6,rtol=0)
            expected=np.float32(np.clip(independently_mapped['encoded_aux_rad']+used,-math.pi/6,math.pi/6))
            np.testing.assert_allclose(requests[i,1::2],expected,atol=1e-6,rtol=0)
            reconstruction.append(dict(frame=i,geometry_supported=rate['supported'],old_correction_rad=old,new_correction_rad=new,
                actual_aux_rad=float(requests[i,1]),old_counterfactual_aux_rad=float(np.float32(np.clip(independently_mapped['encoded_aux_rad']+old,-math.pi/6,math.pi/6))),
                counterfactual_is_same_current_input_arithmetic_not_future_trajectory=True))
        labels=np.load(run/f'maps/frame_{i:03d}.npz')['labels']
        assert not (labels==0).any()
    source=json.loads((run/'source_rows.json').read_text())
    assert [r['frame'] for r in source]==list(range(0,len(states),5))
    source_replay(run,source,'b0')
    clocks=json.loads((run/'camera_clocks.json').read_text());assert len(clocks)==len(states)
    assert all(abs(c['eyes'][e]-c['raw_time_s'])<1e-7 for c in clocks for e in ['left','right'])
    videos={}
    for label in ['stereo_view','third_person']:
        video=cv2.VideoCapture(str(run/(label+'.mp4')));fps=video.get(cv2.CAP_PROP_FPS);n=0
        while video.read()[0]:n+=1
        video.release();assert fps==10. and n==len(states);videos[label]=n
    wall=np.array([r['end_to_end_wall_s'] for r in rows]);assert np.isfinite(wall).all()
    metrics.update(wall_median_s=float(np.median(wall)),wall_p90_s=float(np.percentile(wall,90)),
        requests=len(requests),captures=len(states),physics_samples=len(physics['simulation_time_s']),
        contact_samples=len(contacts),sealed_items=len(manifest['artifact_sha256']),videos=videos,
        observation_gaps=observation_gap_diagnostics(source,rows),correction_reconstruction=reconstruction,unsupported_tracking_count=sum(not r['geometry_supported'] for r in reconstruction))
    return metrics,dict(run=run,requests=requests,physics=physics,episode=episode,identity=identity)

def main():
    out=HERE/'heading_rate_e1_host_assessment_v1.json'
    if out.exists():raise FileExistsError('no repeated performance scoring')
    a,ar=evaluate_arm('b0');b,br=evaluate_arm('candidate')
    assert ar['episode']==br['episode']
    for k in ['camera_rig','pid_config','inner_config','policy_MRO','tracker_MRO','source_sha256','input_sha256']:
        assert ar['identity'][k]==br['identity'][k],k
    n=min(len(ar['requests']),len(br['requests']));forks=np.flatnonzero(np.any(ar['requests'][:n]!=br['requests'][:n],axis=1))
    fork=int(forks[0]) if len(forks) else None;bound=fork if fork is not None else n
    fields=[k for k in ar['physics'] if not k.startswith('raw_')]
    prefix=all(np.array_equal(ar['physics'][k][:5*bound+1],br['physics'][k][:5*bound+1]) for k in fields)
    source_prefix=[]
    for i in range(0,bound+1,5):
        x=dict(np.load(ar['run']/f'source_inputs/frame_{i:03d}.npz'));y=dict(np.load(br['run']/f'source_inputs/frame_{i:03d}.npz'))
        source_prefix.append(dict(frame=i,RGB_exact=all(np.array_equal(x[k],y[k]) for k in ['left_rgb','right_rgb'])))
    changed=any(abs(r['actual_aux_rad']-r['old_counterfactual_aux_rad'])>1e-7 for r in b['correction_reconstruction'])
    gates=navigation_gates(a,b,changed=changed,integrity=True)
    result=dict(arms=dict(b0=a,candidate=b),gates=gates,
        classification=('initial_view_handoff_reject' if any(x['execution_stop_class']=='initial_view_handoff_reject' for x in [a,b]) else ('measured_navigation_accept' if all(gates.values()) else ('measured_navigation_reject' if changed else 'unassessed_incremental_execution_effect'))),
        first_request_fork=fork,nonraw_physics_prefix_exact=prefix,source_RGB_prefix=source_prefix,
        prefix_physics_and_RGB_exact=bool(prefix and all(x['RGB_exact'] for x in source_prefix)),
        full_causal_claim=False,
        TaskH_knownpose_development_only=True,estimated_pose_admitted=False,strict_visual_admitted=False,
        original_rejections_preserved=True,whole_mission_complete=False)
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='arms'},indent=2))

if __name__=='__main__':main()
