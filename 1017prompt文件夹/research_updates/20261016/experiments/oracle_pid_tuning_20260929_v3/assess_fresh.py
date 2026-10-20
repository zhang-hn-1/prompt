from pathlib import Path
import json,numpy as np
from assess_tuning import measure
h=Path(__file__).resolve().parent;b=[measure(h/f'fresh_base_{m}',True) for m in [.5,-1.5]];c=[measure(h/f'fresh_selected_{m}',True) for m in [.5,-1.5]];br=float(np.sqrt(np.mean([r['curvature_rms']**2 for r in b])));cr=float(np.sqrt(np.mean([r['curvature_rms']**2 for r in c])))
checks=dict(improves_10pct=cr<=.9*br,no_tail_gate_regression=all(cw['tail_gate'] or not bw['tail_gate'] for bb,cc in zip(b,c) for bw,cw in zip(bb['windows'],cc['windows'])),finite=all(r['finite'] for r in c),bank_below35=all(r['max_bank_deg']<35 for r in c))
result=dict(baseline=b,candidate=c,baseline_curvature_rms=br,candidate_curvature_rms=cr,improvement_fraction=1-cr/br,checks=checks,all_pass=all(checks.values()));(h/'fresh_assessment.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ['baseline','candidate']},indent=2))
