"""Full sealed65pair validation: RGB-only frozen head, separate known-pose projection."""
from pathlib import Path
import datetime,time,json,hashlib
import numpy as np
from frozen_floor_features import load_frozen,native_probabilities,dense_head_probability
from semantic_floor import project_floor
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    p=json.loads((H/'floor_adapter_fit_protocol.json').read_text());fit=json.loads((H/'floor_adapter_fit.json').read_text());seal=json.loads((H/'semantic_adapter_heldout_seal.json').read_text());assert sha(H/'semantic_adapter_heldout_seal.json')==p['heldout_seal_sha256'];assert fit['optimizer_converged'],'unconverged fit is rejected, not a prediction candidate';assert sha(fit['parameters'])==fit['parameters_sha256']
    for field in ['source_sha256','checkpoint_sha256']:
        for file,value in p[field].items():assert sha(file)==value,file
    for key in ['rgb_only_prediction_input_sha256','known_pose_projection_input_sha256']:
        for file,value in seal[key].items():assert sha(file)==value,file
    z=np.load(fit['parameters']);head=dict(weight=z['weight'],bias=float(z['bias']));model,processor=load_frozen(H/'semantic_floor_model');masks=np.load(p['mask_path']);out=H/'floor_adapter_heldout_predictions';out.mkdir(exist_ok=False);rows=[]
    for i in range(0,321,5):
        rgb=np.load(H/'semantic_adapter_heldout_inputs'/f'frame_{i:03d}.npz');assert set(rgb.files)=={'left_rgb','right_rgb','frame','time_s'};poses=np.load(H/'semantic_adapter_heldout_inputs'/f'frame_{i:03d}_known_pose.npz')
        for eye in ['left','right']:
            started=datetime.datetime.now(datetime.timezone.utc).isoformat();clock=time.perf_counter();packet=dict(rgb=rgb[eye+'_rgb'],time_s=float(rgb['time_s']),frame=i,eye=eye);prob=native_probabilities(packet,model,processor);floorprob=dense_head_probability(prob,packet['rgb'],head);nativefloor=floorprob>=.9
            pred=project_floor(nativefloor,poses[eye+'_camera_usd_world'],masks[eye],**p['projection']);pred['floor_probability_native']=floorprob.astype(np.float32);file=out/f'frame_{i:03d}_{eye}.npz';np.savez_compressed(file,**pred)
            row=dict(frame=i,eye=eye,time_s=packet['time_s'],prediction=str(file),sha256=sha(file),started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),elapsed_cpu_wall_s=time.perf_counter()-clock,projected_pixels=int(pred['confidence'].sum()),raw_semantic_floor_pixels=int(nativefloor.sum()));rows.append(row)
        if i%30==0:print('FRESH_HELDOUT_PREDICTION',i,rows[-2]['projected_pixels'],rows[-1]['projected_pixels'],flush=True)
    (out/'manifest.json').write_text(json.dumps(dict(all_130_saved_before_labels=True,heldout_whole_sequence=True,fit_parameters_sha256=fit['parameters_sha256'],heldout_seal_sha256=p['heldout_seal_sha256'],rows=rows),indent=2)+'\n')
if __name__=='__main__':main()
