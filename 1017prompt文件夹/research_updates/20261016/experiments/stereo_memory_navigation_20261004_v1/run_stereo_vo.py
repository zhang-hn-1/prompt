"""Causal whole historical sequence; sanitized features only, no trajectory labels."""
from pathlib import Path
import datetime,json,hashlib,time
import numpy as np
from stereo_vo import estimate_increment,PoseChain
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def main():
    protocol=json.loads((H/'stereo_vo_protocol.json').read_text())
    for path,value in protocol['source_sha256'].items():assert sha(path)==value,path
    for path,value in protocol['prediction_input_sha256'].items():assert sha(path)==value,path
    inputs=H/'stereo_vo_inputs';out=H/'stereo_vo_predictions';out.mkdir(exist_ok=False)
    calibration=json.loads((inputs/'calibration.json').read_text());K=np.array(calibration['K']);rig=np.array(calibration['right_optical_in_left'])
    expected={eye+'_'+field for eye in ['left','right'] for field in ['xy','desc','size']}|{'frame','time_s'}
    chain=PoseChain();rows=[];previous=None
    for i in range(181):
        with np.load(inputs/f'frame_{i:03d}.npz') as p:
            assert set(p.files)==expected;packet={k:p[k] for k in p.files}
        assert int(packet['frame'])==i;t=float(packet['time_s']);started=now();tick=time.perf_counter()
        if previous is None:result=dict(accepted=True,reason='initial_gauge_only',transform=None);pose=chain.pose.copy()
        else:
            result=estimate_increment(previous,packet,K,rig);pose=chain.update(result['transform'] if result['accepted'] else None)
        row=dict(frame=i,time_s=t,started_utc=started,ended_utc=now(),elapsed_cpu_wall_s=time.perf_counter()-tick,increment=result,
            accumulated_left_optical_to_initial=None if pose is None else pose.tolist(),input_sha256=sha(inputs/f'frame_{i:03d}.npz'))
        path=out/f'frame_{i:03d}.json';path.write_text(json.dumps(row,indent=2,allow_nan=False)+'\n');rows.append(dict(frame=i,path=str(path),sha256=sha(path),ended_utc=row['ended_utc']));previous=packet
        if i%30==0:print('VO',i,result['reason'],result.get('qualified_stereo_landmarks'),flush=True)
    (out/'manifest.json').write_text(json.dumps(dict(saved_utc=now(),rows=rows,all181_serialized_before_truth=True,protocol_sha256=sha(H/'stereo_vo_protocol.json')),indent=2)+'\n')
    print('VO_PREDICTIONS_COMPLETE',now(),flush=True)
if __name__=='__main__':main()
