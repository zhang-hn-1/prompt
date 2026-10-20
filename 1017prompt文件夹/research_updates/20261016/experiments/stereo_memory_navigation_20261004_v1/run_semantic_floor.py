"""One pinned zero-shot RGB classifier; projection independent known-pose layer."""
from pathlib import Path
import json,hashlib,time,datetime
import numpy as np
import torch
from transformers import SegformerFeatureExtractor,SegformerConfig,SegformerForSemanticSegmentation
from semantic_floor import validate_rgb_packet,upright_rgb,native_floor_mask,project_floor
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def infer(packet,model,processor,floor_id):
    validate_rgb_packet(packet);up=upright_rgb(packet['rgb']);inputs=processor(images=up,return_tensors='pt')
    with torch.inference_mode():
        logits=model(**inputs).logits;logits=torch.nn.functional.interpolate(logits,size=up.shape[:2],mode='bilinear',align_corners=False);prob=logits.softmax(dim=1)[0,floor_id].numpy();labels=logits.argmax(dim=1)[0].numpy()
    return native_floor_mask(prob,labels==floor_id,.9),np.rot90(prob,-1).copy(),np.rot90(labels,-1).copy()
def main():
    p=json.loads((H/'semantic_floor_protocol.json').read_text());out=H/'semantic_floor_predictions';out.mkdir(exist_ok=False)
    for field in ['source_sha256','prediction_input_sha256','checkpoint_sha256']:
        for file,value in p[field].items():assert sha(file)==value,file
    torch.set_num_threads(1);torch.manual_seed(0);modelpath=H/'semantic_floor_model';configuration=SegformerConfig.from_json_file(str(modelpath/'config.json'));assert configuration.id2label[p['floor_id']]=='floor'
    model=SegformerForSemanticSegmentation(configuration);weights=torch.load(modelpath/'pytorch_model.bin',map_location='cpu',weights_only=True);model.load_state_dict(weights,strict=True);model.eval();processor=SegformerFeatureExtractor.from_pretrained(str(modelpath),local_files_only=True);masks=np.load(p['mask_path']);rows=[]
    for i in p['frames']:
        rgb=np.load(H/'semantic_floor_inputs'/f'frame_{i:03d}.npz');assert set(rgb.files)=={'left_rgb','right_rgb','frame','time_s'};poses=np.load(H/'rolling_online_run/known_poses'/f'frame_{i:03d}.npz')
        for eye in ['left','right']:
            started=datetime.datetime.now(datetime.timezone.utc).isoformat();clock=time.perf_counter();packet=dict(rgb=rgb[eye+'_rgb'],frame=i,time_s=float(rgb['time_s']),eye=eye)
            floor,prob,label=infer(packet,model,processor,p['floor_id']);prediction=project_floor(floor,poses[eye+'_camera_usd_world'],masks[eye],**p['projection']);prediction.update(floor_probability_native=prob,semantic_label_native=label)
            path=out/f'frame_{i:03d}_{eye}.npz';np.savez_compressed(path,**prediction)
            row=dict(frame=i,eye=eye,time_s=packet['time_s'],started_utc=started,ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),prediction=str(path),sha256=sha(path),elapsed_cpu_wall_s=time.perf_counter()-clock,semantic_floor_pixels=int(floor.sum()),projected_floor_pixels=int(prediction['confidence'].sum()));rows.append(row);print('SEMANTIC_FLOOR',i,eye,row['semantic_floor_pixels'],row['projected_floor_pixels'],flush=True)
    (out/'manifest.json').write_text(json.dumps(dict(all_eight_saved_before_truth=True,rows=rows,role=p['role']),indent=2)+'\n')
if __name__=='__main__':main()
