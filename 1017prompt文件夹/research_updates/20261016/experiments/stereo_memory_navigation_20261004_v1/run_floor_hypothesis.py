"""Fixed four RGB known-pose predictions, no labels/control/map writes."""
from pathlib import Path
import json,hashlib,datetime,time
import numpy as np
from floor_hypothesis import predict_floor
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    p=json.loads((H/'floor_hypothesis_protocol.json').read_text());out=H/'floor_hypothesis_predictions';out.mkdir(exist_ok=False);rows=[]
    for field in ['source_sha256','prediction_input_sha256']:
        for file,value in p[field].items():assert sha(file)==value,file
    masks=np.load(p['mask_path'])
    for number in p['frames']:
        started=datetime.datetime.now(datetime.timezone.utc).isoformat();clock=time.perf_counter()
        def packet(i,eye):
            z=np.load(H/'floor_hypothesis_inputs'/f'frame_{i:03d}.npz');assert set(z.files)=={'left_rgb','right_rgb','left_camera_usd_world','right_camera_usd_world','frame','time_s'}
            return dict(frame=i,time_s=float(z['time_s']),eye=eye,rgb=z[eye+'_rgb'],pose=z[eye+'_camera_usd_world'],mask=masks[eye])
        reference=packet(number,'left');sources=[packet(number,'right')]
        if number>=5:sources.extend([packet(number-5,eye) for eye in ['left','right']])
        prediction=predict_floor(reference,sources,**p['parameters']);path=out/f'frame_{number:03d}.npz';np.savez_compressed(path,**prediction)
        row=dict(frame=number,time_s=reference['time_s'],prediction=str(path),sha256=sha(path),started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),elapsed_cpu_wall_s=time.perf_counter()-clock,output_pixels=int(prediction['confidence'].sum()),sources=[dict(frame=s['frame'],eye=s['eye'],time_s=s['time_s'],bit=1<<i) for i,s in enumerate(sources)])
        rows.append(row);print('FLOOR_HYPOTHESIS',number,row['output_pixels'],flush=True)
    (out/'manifest.json').write_text(json.dumps(dict(all_four_saved_before_labels=True,rows=rows,role=p['role']),indent=2)+'\n')
if __name__=='__main__':main()
