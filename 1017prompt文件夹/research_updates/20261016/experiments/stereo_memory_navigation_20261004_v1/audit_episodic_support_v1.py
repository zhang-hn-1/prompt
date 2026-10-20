"""Closed 30 saved predictions only: classify support gaps, no query/model mutation."""
from pathlib import Path
import json,hashlib,math
import numpy as np
H=Path(__file__).resolve().parent;O=H/'episodic_execution_memory_run_v1'
if __name__=='__main__':
 predictions=json.loads((O/'predictions.json').read_text());lib=np.load(O/'library.npz');rows=[];counts={}
 for p in predictions:
  r=dict(episode=p['episode'],frame=p['frame'],saved_status=p['status'])
  if p['status']=='unavailable_anchor':cause='unavailable_anchor'
  else:
   difference=np.abs(lib['program']-np.array(p['proposal']['commands']));mask=(difference[:,:,0]<=.01+1e-12).all(axis=1)&(difference[:,:,1]<=math.radians(2)+1e-12).all(axis=1)
   r['program_matching_entries']=int(mask.sum())
   if mask.any():
    distances=np.sqrt(np.mean((lib['feature'][mask]-np.array(p['feature']))**2,axis=1));r['nearest_matching_state_rms']=float(distances.min());cause='supported' if distances.min()<=.25 else 'state_distance_outside_frozen_domain'
   else:cause='no_library_program_in_frozen_tolerance'
   assert (cause=='supported')==(p['status']=='supported')
  r['cause']=cause;counts[cause]=counts.get(cause,0)+1;rows.append(r)
 out=H/'episodic_execution_support_diagnostic_v1.json';assert not out.exists();out.write_text(json.dumps(dict(inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [O/'predictions.json',O/'library.npz']},fixed_saved_anchors=30,counts=counts,rows=rows,new_queries=0,new_teacher_labels=0,parameters_changed=False,no_admission=True),indent=2)+'\n');print(counts)
