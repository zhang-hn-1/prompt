"""One source-derived appearance hypothesis; seal before labels, no depth inference."""
from pathlib import Path
import json,datetime,hashlib
import numpy as np,cv2
H=Path(__file__).resolve().parent;D=H/'source_event_handoff_C1_run_v1';O=H/'qualified_appearance_later_motion_v1';O.mkdir(exist_ok=False);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();source=np.load(D/'qualified_sensor_packets/source_150.npz');now=float(source['time_s']);assert int(source['frame'])==150
haz=(source['world'][:,2]>.05)&(source['world'][:,2]<=1.5);assert haz.sum()>=100
colors=[];inputs={};r=np.load(D/'online_inputs/frame_150.npz')
for eye,k in [('left','query_pixel_id'),('right','partner_pixel_id')]:
 ids=source[k][haz];lab=cv2.cvtColor(np.ascontiguousarray(r[eye+'_rgb'][:,:,:3]),cv2.COLOR_RGB2LAB).astype(float);colors.append(lab.reshape(-1,3)[ids])
c=np.vstack(colors);center=np.median(c,axis=0);scale=np.maximum(8.,1.4826*np.median(np.abs(c-center),axis=0));rows=[]
def validate_before_payload(record,clock):
 t=float(record['time_s'])
 if not np.isfinite(t) or t>clock+1e-9:raise ValueError('past/current exposure only before RGB payload')
 return record['rgb']
class Trap(dict):
 def __getitem__(self,k):
  if k!='time_s':raise AssertionError('future RGB payload touched')
  return super().__getitem__(k)
traps=0
for _ in range(10):
 try:validate_before_payload(Trap(time_s=now+1),now)
 except ValueError:traps+=1
 else:raise AssertionError('future exposure accepted')
for i in range(155,266,5):
 rp=D/'online_inputs'/f'frame_{i:03d}.npz';mp=D/'masks'/f'frame_{i:03d}.npz';rgb=np.load(rp);m=np.load(mp)
 for p in [rp,mp]:inputs[str(p)]=sha(p)
 for eye in ['left','right']:
  query_clock=float(rgb['time_s']);assert query_clock>=now;image=validate_before_payload(dict(time_s=rgb['time_s'],rgb=rgb[eye+'_rgb']),query_clock);lab=cv2.cvtColor(np.ascontiguousarray(image[:,:,:3]),cv2.COLOR_RGB2LAB).astype(float);score=np.linalg.norm((lab-center)/scale,axis=-1);prediction=(score<=3)&~m[eye];f=O/f'{eye}_{i:03d}.npz';np.savez_compressed(f,potential_hazard_mask=prediction,exposure_time_s=float(rgb['time_s']),hypothesis_birth_time_s=query_clock,appearance_model_birth_time_s=now,frame=i,eye=eye,not_depth=True,not_observed_occupancy=True)
  rows.append(dict(frame=i,eye=eye,exposure_time_s=float(rgb['time_s']),hypothesis_birth_time_s=query_clock,appearance_model_birth_time_s=now,predicted_pixels=int(prediction.sum()),file=str(f),sha256=sha(f)))
inputs[str(D/'qualified_sensor_packets/source_150.npz')]=sha(D/'qualified_sensor_packets/source_150.npz')
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision_clock_s=now,seed_frame=150,seed_points=int(haz.sum()),Lab_center=center.tolist(),Lab_scale=scale.tolist(),max_standardized_distance=3.,scale_floor=8.,MAD_factor=1.4826,rows=rows,input_sha256=inputs,future_payload_rejections=traps,all_development_not_fresh=True,renderer_labels_read=False,not_stereo_depth_or_occupancy=True,code_sha256=sha(__file__),plan_sha256=sha(H.parent.parent/'docs/superpowers/plans/2026-10-07-appearance-later-motion-regression.md'))
(O/'prediction_seal_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ['input_sha256','rows']},indent=2))
