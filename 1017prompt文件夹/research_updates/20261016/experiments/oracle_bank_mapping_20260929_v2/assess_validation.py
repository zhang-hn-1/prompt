from pathlib import Path
import json,numpy as np
from measurement import geometry,bank
h=Path(__file__).resolve().parent;rows=[]
for p in sorted(h.glob('validation_*/physics.npz')):
 z=np.load(p);main=json.loads((p.parent/'config.json').read_text())['main'];t=z['simulation_time_s'];v,k=geometry(z['position_world_xy']);b=np.degrees(bank(z['root_quaternion_world_wxyz']));requests=json.loads((p.parent/'mapping_queries.json').read_text())
 for end,target in [(20,.4),(35,-.4),(50,.7),(65,-.7)]:
  mask=(t>=end-5)&(t<=end-.5);valid=mask&np.isfinite(k);actual=float(np.nanmedian(k[valid])*np.sign(main));error=abs(actual-target);threshold=max(.2,.3*abs(target));q=[r for r in requests if end-5<=r['block']*.1<=end-.5]
  rows.append(dict(case=p.parent.name,main=main,requested_curvature=target,actual_curvature_body=actual,error=error,threshold=threshold,sign_match=actual*target>0,valid_fraction=float(valid.sum()/mask.sum()),actual_speed_m_s=float(np.median(v[valid])),actual_bank_deg=float(np.median(b[valid])),requested_bank_deg=float(np.degrees(np.median([r['bank_rad'] for r in q]))),curvature_clamp_fraction=float(np.mean([r['curvature_clamped'] for r in q])),accepted=bool(actual*target>0 and error<=threshold and valid.sum()/mask.sum()>=.9)))
result=dict(rows=rows,all_pass=len(rows)==8 and all(r['accepted'] for r in rows),independent_whole_sequence=True,no_refit=True)
(h/'validation_assessment.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
