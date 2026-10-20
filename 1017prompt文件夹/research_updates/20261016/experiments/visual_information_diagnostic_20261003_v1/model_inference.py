"""Official metric model cores and preprocessing; RGB+known K only."""
from pathlib import Path
import json,sys,time
import numpy as np
import torch
import torch.nn.functional as F
from measurement import upright_calibration,restore_depth
H=Path(__file__).resolve().parent
sys.path[:0]=[str(H/'vendor'),str(H/'third_party/da3/src'),str(H/'third_party/unidepth')]
FOCAL=16.45/18*640
K=np.array([[FOCAL,0,319.5],[0,FOCAL,239.5],[0,0,1.]])
_,UP_K,_=upright_calibration(np.zeros((1,2)),K,640)

class MetricModel:
    online_input_permissions=frozenset({'current_rgb','calibration'})
    def __init__(self,name):
        from safetensors.torch import load_file
        self.name=name;self.device='cuda';torch.set_num_threads(2)
        if name=='da3_metric':
            from depth_anything_3.cfg import create_object
            from omegaconf import OmegaConf
            from depth_anything_3.utils.io.input_processor import InputProcessor
            cfg=json.loads((H/'models/da3_metric/config.json').read_text())
            self.model=create_object(OmegaConf.create(cfg['config']))
            state=load_file(str(H/'models/da3_metric/model.safetensors'))
            state={k.removeprefix('model.'):v for k,v in state.items()}
            self.model.load_state_dict(state,strict=True);self.preprocess=InputProcessor()
        elif name=='unidepth_v2':
            from unidepth.models import UniDepthV2
            cfg=json.loads((H/'models/unidepth_v2/config.json').read_text())
            self.model=UniDepthV2(cfg)
            self.model.load_state_dict(load_file(str(H/'models/unidepth_v2/model.safetensors')),strict=True)
            self.model.resolution_level=0
        else:raise ValueError(name)
        self.model.eval().to(self.device)

    @torch.inference_mode()
    def predict(self,rgb):
        if np.asarray(rgb).shape!=(480,640,3):raise ValueError('fixed rig RGB shape required')
        up=np.ascontiguousarray(np.rot90(rgb));torch.cuda.synchronize();start=time.perf_counter()
        if self.name=='da3_metric':
            inputs,_,ixt=self.preprocess([up],intrinsics=UP_K[None].astype('float32'),process_res=504,num_workers=1,sequential=True)
            if inputs.ndim==4:inputs=inputs[None]
            inputs=inputs.to(self.device)
            with torch.autocast('cuda',dtype=torch.float16):out=self.model(inputs)
            raw=out['depth'].float().reshape(1,1,*out['depth'].shape[-2:])
            # Official metric scaling, focal at actual processed resolution; no GT alignment.
            focal=float((ixt[...,0,0]+ixt[...,1,1]).mean()/2)
            depth=F.interpolate(raw*focal/300,size=(640,480),mode='bilinear',align_corners=False)[0,0].cpu().numpy()
            confidence=np.full_like(depth,np.nan)
        else:
            inputs=torch.from_numpy(up).permute(2,0,1).to(self.device)
            with torch.autocast('cuda',dtype=torch.float16):
                out=self.model.infer(inputs,torch.from_numpy(UP_K).float().unsqueeze(0).to(self.device))
            depth=out['depth'][0,0].float().cpu().numpy();confidence=out['confidence'][0,0].float().cpu().numpy()
        torch.cuda.synchronize();elapsed=time.perf_counter()-start
        return restore_depth(depth),restore_depth(confidence),elapsed

if __name__=='__main__':
    import argparse,hashlib
    p=argparse.ArgumentParser();p.add_argument('--model',required=True);p.add_argument('--limit',type=int,default=0);a=p.parse_args()
    manifest=json.loads((H/'offline_manifest.json').read_text());model=MetricModel(a.model);records=[]
    for i,row in enumerate(manifest['samples']):
        if a.limit and i>=a.limit:break
        outdir=H/'predictions'/a.model/row['sequence'];outdir.mkdir(parents=True,exist_ok=True)
        dest=outdir/Path(row['rgb_file']).name
        if dest.exists():continue
        with np.load(row['rgb_file']) as f:
            save={};times=[]
            for eye in ('left','right'):
                depth,confidence,latency=model.predict(f[eye+'_rgb']);save[eye+'_depth_m']=depth;save[eye+'_confidence']=confidence;times.append(latency)
        np.savez_compressed(dest,**save)
        records.append(dict(sequence=row['sequence'],frame=dest.name,eye_seconds=times,pair_seconds=sum(times),gpu_max_allocated_bytes=torch.cuda.max_memory_allocated()))
        (H/(a.model+'_inference_records.json')).write_text(json.dumps(records,indent=2)+'\n')
        print(a.model,i+1,len(manifest['samples']),row['sequence'],dest.name,[round(t,3) for t in times],flush=True)
