"""Offline labels after all181 pose predictions have serialized."""
from pathlib import Path
import datetime,json,hashlib,collections
import numpy as np
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def optical(usd):
    T=np.asarray(usd,float).copy();T[:3,:3]=T[:3,:3]@np.diag([1.,-1.,-1.]);return T
def errors(pred,actual):
    delta=np.linalg.inv(actual)@np.asarray(pred);angle=np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1))
    return [float(np.linalg.norm(delta[:3,3])),float(np.degrees(angle))]
def main():
    p=json.loads((H/'stereo_vo_protocol.json').read_text());m=json.loads((H/'stereo_vo_predictions/manifest.json').read_text());started=now()
    assert m['all181_serialized_before_truth'] and len(m['rows'])==181
    assert all(r['ended_utc']<started for r in m['rows'])
    for path,value in p['evaluation_input_sha256'].items():assert sha(path)==value,path
    for r in m['rows']:assert sha(r['path'])==r['sha256']
    truth=[optical(np.load(H/'active_observation_run/known_poses'/f'frame_{i:03d}.npz')['left_camera_usd_world']) for i in range(181)]
    records=[];layers=collections.Counter();valid_global=[];relative=[]
    for r in m['rows']:
        row=json.loads(Path(r['path']).read_text());i=r['frame'];inc=row['increment'];layers[inc['reason']]+=1
        absolute=None;pair=None
        if row['accumulated_left_optical_to_initial'] is not None:
            absolute=errors(row['accumulated_left_optical_to_initial'],np.linalg.inv(truth[0])@truth[i]);valid_global.append(absolute)
        if i>0 and inc['accepted']:
            pair=errors(inc['transform'],np.linalg.inv(truth[i])@truth[i-1])
        if i>0 and row['time_s']>=6.-1e-6:relative.append(pair)
        records.append(dict(frame=i,time_s=row['time_s'],reason=inc['reason'],relative_error_m_deg=pair,absolute_error_m_deg=absolute,
            stereo_landmarks=inc.get('qualified_stereo_landmarks'),temporal_matches=inc.get('temporal_matches'),joint_matches=inc.get('joint_matches'),elapsed_cpu_wall_s=row['elapsed_cpu_wall_s']))
    finite=np.array([r for r in relative if r is not None],float).reshape(-1,2);glob=np.array(valid_global,float).reshape(-1,2)
    fraction=float(sum(r is not None and r[0]<=.05 and r[1]<=2 for r in relative)/len(relative))
    relp90=np.percentile(finite,90,axis=0).tolist() if len(finite) else None
    globalcoverage=len(glob)/181;grms=float(np.sqrt(np.mean(glob[:,0]**2)));gpeak=float(glob[:,0].max());grot=float(np.percentile(glob[:,1],90))
    gates=dict(moving_all_correct_fraction=fraction>=.9,relative_p90=relp90 is not None and relp90[0]<=.05 and relp90[1]<=2,
        global_coverage=globalcoverage>=.9,global_translation_rms=grms<=.05,global_translation_peak=gpeak<=.1,global_rotation_p90=grot<=2)
    result=dict(started_utc=started,completed_utc=now(),frames=181,pair_attempts=180,sequence='active_e1_original18s',historical_not_fresh=True,
        moving_pair_denominator=len(relative),moving_finite_pairs=len(finite),moving_all_correct_fraction=fraction,relative_error_p90_m_deg=relp90,
        global_valid_frames=len(glob),global_coverage=globalcoverage,global_translation_rms_m=grms,global_translation_peak_m=gpeak,global_rotation_p90_deg=grot,
        failure_layers=dict(layers),gates=gates,pose_candidate_accepted=all(gates.values()),depth_admitted=False,navigation_admitted=False,new_physics=0,new_networks=0,new_fits=0,
        labels='GT world camera poses used only after all predictions saved; initial pose for evaluation gauge alignment only',records=records,whole_research_complete=False)
    (H/'stereo_vo_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print('VO',result['pose_candidate_accepted'],result['failure_layers'],len(finite),globalcoverage,flush=True)
if __name__=='__main__':main()
