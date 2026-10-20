"""Causal frozen ORB regression on every capture; no offline label loading."""
from pathlib import Path
import datetime,hashlib,json,time
import numpy as np
from feature_tracks import extract
from active_observation import guarded_predict,SafeFeatureMemory
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
def main():
    p=json.loads((H/'active_observation_protocol.json').read_text());status=json.loads((H/'active_observation_status.json').read_text());assert status['exit_code']==0
    for f,s in p['source_sha256'].items():assert sha(f)==s,f
    inference=json.loads((H/'active_feature_protocol.json').read_text())
    for f,s in inference['source_sha256'].items():assert sha(f)==s,f
    root=H/'active_observation_run';out=H/'active_feature_runs';out.mkdir(exist_ok=False);rows=[];history=[];memory=SafeFeatureMemory(10)
    masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');K=np.array([[584.8888888888889,0,319.5],[0,584.8888888888889,239.5],[0,0,1.]])
    for i in range(181):
        rgb_path=root/'online_inputs'/f'frame_{i:03d}.npz';pose_path=root/'known_poses'/f'frame_{i:03d}.npz';raw=np.load(rgb_path);known=np.load(pose_path)
        assert set(raw.files)=={'left_rgb','right_rgb','time_s'} and set(known.files)=={'left_camera_usd_world','right_camera_usd_world','time_s'}
        t=float(raw['time_s']);assert t==float(known['time_s']);pair=[];features={};start=time.perf_counter()
        for eye in ['left','right']:
            xy,desc,size=extract(raw[eye+'_rgb'],masks[eye]);pose=known[eye+'_camera_usd_world'];pair.append(dict(frame=i,time_s=t,eye=eye,xy=xy,desc=desc,size=size,pose=pose))
            for k,v in [('xy',xy),('desc',desc),('size',size),('pose',pose)]:features[eye+'_'+k]=v
        fp=out/f'features_{i:03d}.npz';np.savez_compressed(fp,**features,frame=i,time_s=t)
        current=guarded_predict(pair[0],[pair[1]],K);causal=guarded_predict(pair[0],[pair[1]]+history,K);memory.update(causal['points'],t,0.)
        result=dict(sequence='active_e1',frame=i,time_s=t,current=current,causal=causal,memory=memory.snapshot(t),pose_sigma_m=0.,seconds=time.perf_counter()-start,features_sha256=sha(fp),rgb_source_sha256=sha(rgb_path),known_pose_source_sha256=sha(pose_path),source_packet_fields=sorted(pair[0]))
        path=out/f'prediction_{i:03d}.json';path.write_text(json.dumps(result,indent=2)+'\n');rows.append(dict(frame=i,time_s=t,sha256=sha(path),features_sha256=sha(fp),queries=current['query_count'],current_retained=len(current['points']),causal_retained=len(causal['points']),memory_records=len(result['memory']),seconds=result['seconds']))
        history=(history+pair)[-6:]
        if i%30==0:print('FEATURE',i,current['query_count'],len(current['points']),len(causal['points']),flush=True)
    manifest=dict(saved_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),labels_not_loaded=True,rows=rows,source_sha256={str(f):sha(f) for f in [Path(__file__),H/'feature_tracks.py',H/'active_observation.py',H/'active_observation_protocol.json']},scope='all181current/last3strictpast knownpose pairs; no GTdepth/geometry/future input, old candidate rejected remains')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('ACTIVE_FEATURE_COMPLETE',len(rows),manifest['saved_utc'])
if __name__=='__main__':main()
