"""Post-hoc transient measurement, not used to relax or tune selection gates."""
from pathlib import Path
import json,numpy as np
from measurement import geometry
h=Path(__file__).resolve().parent;rows=[]
for p in sorted(h.glob('validation_*/physics.npz')):
 z=np.load(p);t=z['simulation_time_s'];main=json.loads((p.parent/'config.json').read_text())['main'];v,k=geometry(z['position_world_xy']);k=k*np.sign(main)
 for start,end,target in [(5,20,.4),(20,35,-.4),(35,50,.7),(50,65,-.7)]:
  mask=(t>=start+.02)&(t<=end-.5)&np.isfinite(k);ix=np.flatnonzero(mask);threshold=max(.2,.3*abs(target));outside=abs(k[ix]-target)>threshold
  final_settling=(float(t[ix[np.flatnonzero(outside)[-1]]]-start+.02) if np.any(outside) else 0.)
  rows.append(dict(main=main,start_s=start,requested_curvature=target,last_exit_then_stays_in_band_s=final_settling,band_halfwidth=threshold,wrong_curvature_sign_duration_s=float(np.sum(k[ix]*target<0)*.02),maximum_abs_curvature_error=float(np.max(abs(k[ix]-target)))))
result=dict(definition='Last exit from requested curvature +/- validation band until phase end-.5s; centered31pt(.6s support) offline geometry, timing approximate, not online criterion',rows=rows)
(h/'transient_assessment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
