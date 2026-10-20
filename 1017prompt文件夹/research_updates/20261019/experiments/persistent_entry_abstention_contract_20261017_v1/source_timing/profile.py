"""Exactly original baseline outputs; timing wrappers only. No parameter/decision change."""
from pathlib import Path
import sys,json,hashlib,time,datetime
D=Path(__file__).resolve().parent;W=D.parents[2];S=W/'experiments/four_camera_weak_baseline_20261017_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
cfg=json.loads((D/'protocol.json').read_text())
for p,h in cfg['input_sha256'].items():assert sha(p)==h,p
for p,h in cfg['code_sha256'].items():assert sha(p)==h,p
sys.path.insert(0,str(S))
from source import FourSGBM,NAMES
import numpy as np
class TimedCompute:
 def __init__(self,matcher,rows,key):self.matcher=matcher;self.rows=rows;self.key=key
 def compute(self,a,b):
  t=time.perf_counter();r=self.matcher.compute(a,b);self.rows.append(dict(stage=self.key,wall_s=time.perf_counter()-t));return r
init=time.perf_counter();m=FourSGBM();initialization=time.perf_counter()-init;records=[];runs=[]
left,right=m.left,m.right;original_masks=m.masks
for f in cfg['frames']:
 rows=[];m.left=TimedCompute(left,rows,'left_compute');m.right=TimedCompute(right,rows,'right_compute')
 def timed_masks(joints):
  t=time.perf_counter();r=original_masks(joints);rows.append(dict(stage='dynamic_CAD_masks',wall_s=time.perf_counter()-t));return r
 m.masks=timed_masks
 with np.load(f['raw']) as a:
  rgb={n:a[n+'_rgb'] for n in NAMES};joints=a['joints_current']
 t=time.perf_counter();pred,internal=m.predict(rgb,joints);total=time.perf_counter()-t
 with np.load(f['prediction']) as old:
  for n in NAMES:
   for key,suffix in [('depth','depth'),('valid','valid'),('disparity','disparity'),('CAD','CAD')]:
    actual=pred[n][key];reference=old[n+'_'+suffix]
    assert actual.dtype==reference.dtype and actual.shape==reference.shape and actual.tobytes()==reference.tobytes(),(f['frame'],n,key)
 summed=sum(v['wall_s'] for v in rows);record=dict(frame=f['frame'],source_wall_s=total,internal_wall_s=internal,dynamic_CAD_masks_s=sum(v['wall_s'] for v in rows if v['stage']=='dynamic_CAD_masks'),four_stereo_compute_s=sum(v['wall_s'] for v in rows if v['stage']!='dynamic_CAD_masks'),other_predict_s=total-summed,calls=rows,all16_output_arrays_byte_exact=True,raw_sha256=f['raw_sha256'],prediction_sha256=f['prediction_sha256']);runs.append(record);print('PROFILE',json.dumps(record),flush=True)
result=dict(complete=True,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),initialization_s=initialization,rows=runs,source_invocations=2,disparity_compute_calls=8,new_model=0,fit=0,physics=0,navigation=0,scope='One invocation each frozenframe0and90, ordinary warmed-state ordering as declared; two descriptive samples only, no realtime, speedup or full-domain performance claim. Original source and outputs unmodified; actual same-SGBM source recomputation2 for output-equality timing attribution only.')
(D/'result.json').write_text(json.dumps(result,indent=2)+'\n')
