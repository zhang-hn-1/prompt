from pathlib import Path
import json,hashlib,numpy as np
from assess_tuning import measure
h=Path(__file__).resolve().parent;old=h.parent/'oracle_pid_validation_20260929_v4';base=[measure(old/f'fresh_selected_{m}',True) for m in [.5,-1.5]]
def rms(rows,key):return float(np.sqrt(np.mean([r[key]**2 for r in rows])))
result={'baseline':base,'candidates':[]};bk=rms(base,'curvature_rms');bb=rms(base,'bank_rms_deg')
for tag in ['limit060','limitoff']:
 rows=[measure(h/f'{tag}_{m}',True) for m in [.5,-1.5]];ck=rms(rows,'curvature_rms');cb=rms(rows,'bank_rms_deg');checks=dict(curvature_improves_10pct=ck<=.9*bk,bank_not_worse10pct=cb<=1.1*bb,all_tail_pass=all(r['all_tail_pass'] for r in rows),finite=all(r['finite'] for r in rows),bank_below35=all(r['max_bank_deg']<35 for r in rows));result['candidates'].append(dict(tag=tag,rows=rows,curvature_rms=ck,bank_rms_deg=cb,improvement_fraction=1-ck/bk,checks=checks,accepted=all(checks.values())))
result['baseline_curvature_rms']=bk;result['baseline_bank_rms_deg']=bb
identity=json.loads((h/'dynamic_identity.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==v for p,v in identity.items());result['dynamic_frozen_identity_unchanged']=True
for group in [base]+[r['rows'] for r in result['candidates']]:
 for row in group:
  p=old/row['case'] if row['case'].startswith('fresh_') else h/row['case'];z=np.load(p/'physics.npz');v=abs(z['joint_velocity_rad_s'][:,1]);row['actual_aux_speed_p95_rad_s']=float(np.percentile(v,95));row['actual_aux_speed_peak_rad_s']=float(v.max())
(h/'limits_assessment.json').write_text(json.dumps(result,indent=2));print('BASE',bk,bb)
for r in result['candidates']:print(r['tag'],r['curvature_rms'],r['improvement_fraction'],r['checks'])
