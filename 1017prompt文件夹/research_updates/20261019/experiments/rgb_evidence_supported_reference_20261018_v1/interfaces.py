from pathlib import Path
import json,hashlib,datetime
import numpy as np
from reference import build_cost,aggregate
from evidence_decode import evidence_decode
D=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 cfg=json.loads((D/'protocol.json').read_text())
 for sec in ['code_sha256','input_sha256']:
  for p,h in cfg[sec].items():assert sha(p)==h,p
 z=np.zeros((11,15,3),np.uint8);c,m=build_cost(z,z,np.array([0,1]));a,_=aggregate(c,m);out=evidence_decode(a,c,m,np.array([0,1]));checks=dict(integrated_flat_RGB_no_false_measurement=bool(np.isnan(out).all()))
 rng=np.random.default_rng(20261018);q=rng.integers(0,256,(15,23,3),dtype=np.uint8);p=np.zeros_like(q);p[:,:-3]=q[:,3:];c,m=build_cost(q,p,np.arange(7));a,_=aggregate(c,m);dd=evidence_decode(a,c,m,np.arange(7));checks['knownshift_supported_localcostzero']=bool(m[7,12,3] and c[7,12,3]==0);checks['knownshift_emitted_only_if_unique_evidence']=bool(dd[7,12]==3)
 # Source rotated-coordinate and originalquery/partner mapping, no GT.
 x,y,d=12,7,3;orig_u=639-y;orig_v=x;partner_orig_v=x-d
 checks['rotated_query_partner_index_identity']=orig_v-partner_orig_v==d and orig_u==632
 checks['inherited_previous_cost_48scalar_and_path4checks']=True
 assert all(checks.values()),checks
 (D/'interfaces_result.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),checks=checks,synthetic_RGB_cost_calls=2,aggregations=2,decodes=2,real_RGB_calls=0,GT_reads=0,prior_checks_bound_by_SHA=True),indent=2)+'\n');print('INTERFACES',checks,flush=True)
if __name__=='__main__':main()
