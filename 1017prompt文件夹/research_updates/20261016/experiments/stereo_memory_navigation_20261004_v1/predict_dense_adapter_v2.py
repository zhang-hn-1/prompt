"""Same B0/decoder candidate RGB-only complete whole-sequence predictions."""
from pathlib import Path
import json,hashlib,datetime,time
import numpy as np
import torch
from fast_stereo import FastStereo
from dense_adapter import PREFIXES
H=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()

def main():
 p=json.loads((H/'dense_adapter_fit_v2_protocol.json').read_text());fit=json.loads((H/'dense_adapter_fit_v2_status.json').read_text());assert fit['status']=='finished' and fit['attempts']==300
 for kind in ['source_sha256','input_sha256']:
  for f,d in p[kind].items():assert sha(f)==d,f
 assert sha(fit['parameters_path'])==fit['parameters_sha256'];out=H/'dense_adapter_v2_predictions';out.mkdir(exist_ok=False)
 masks=np.load(p['mask']);rows=[];status=H/'dense_adapter_v2_prediction_status.json';started=now();status.write_text(json.dumps(dict(start_utc=started,status='running'),indent=2)+'\n')
 for arm in ('b0','adapter'):
  model=FastStereo()
  if arm=='adapter':
   state=torch.load(fit['parameters_path'],map_location='cpu',weights_only=True);assert set(state)==set(fit['trainable_names']) and all(n.startswith(PREFIXES) for n in state)
   original=model.model.state_dict();assert all(n in original and v.shape==original[n].shape for n,v in state.items());result=model.model.load_state_dict(state,strict=False);assert not result.unexpected_keys
  folder=out/arm;folder.mkdir()
  for frame in range(0,321,5):
   source=H/'dense_adapter_heldout_rgb'/f'frame_{frame:03d}.npz';packet=np.load(source);assert set(packet.files)=={'left_rgb','right_rgb','time_s'}
   clock=time.perf_counter();pred=model.predict(packet['left_rgb'],packet['right_rgb'],masks['left'],masks['right']);elapsed=time.perf_counter()-clock
   dest=folder/source.name;np.savez_compressed(dest,**pred);rows.append(dict(arm=arm,frame=frame,time_s=float(packet['time_s']),path=str(dest),sha256=sha(dest),seconds=elapsed));print('PRED',arm,frame,elapsed,flush=True)
  del model;torch.cuda.empty_cache()
 assert len(rows)==130
 (out/'manifest.json').write_text(json.dumps(dict(saved_utc=now(),rows=rows,labels_not_loaded=True,pose_not_loaded=True,fit_protocol_sha256=sha(H/'dense_adapter_fit_v2_protocol.json'),fit_weight_sha256=fit['parameters_sha256'],source_layer='RGBonlynoGT/pose',eager_both_arms=True,control_consumed=False),indent=2)+'\n');status.write_text(json.dumps(dict(start_utc=started,end_utc=now(),status='finished',predictions=130),indent=2)+'\n')
if __name__=='__main__':main()
