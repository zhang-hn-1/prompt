"""Single frozen failing fixture, no rule/parameter/data/source variation."""
from pathlib import Path
import hashlib,json,datetime
import numpy as np
from reference import build_cost,aggregate,decode
D=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 cfg=json.loads((D/'analysis_protocol.json').read_text())
 for p,h in cfg['sha256'].items():assert sha(p)==h,p
 z=np.zeros((11,15,3),np.uint8);ds=np.array([0,1]);c,m=build_cost(z,z,ds);a,p=aggregate(c,m,True);w=decode(a,ds);y,x=5,6
 assert np.array_equal(c[y,x],np.array([0.,0.])) and m[y,x].all()
 assert np.array_equal(p['left'][y,x],np.array([0.,600.]))
 assert np.array_equal(p['right'][y,x],np.array([0.,0.])) and np.array_equal(p['top'][y,x],np.array([0.,0.]))
 assert np.array_equal(m[y,4],np.array([True,False])) and w[y,x]==0
 np.savez_compressed(D/'failure_analysis_arrays.npz',cost=c,support=m,total=a,decoded=w,**{k+'_path':v for k,v in p.items()})
 v=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),status='PREDECLARED_FLAT_UNKNOWN_GATE_FAILED_CAUSE_CONFIRMED_NO_REALRGB',fixed_fixture=dict(shape=[11,15,3],RGB_value=0,disparities=[0,1],pixel_yx=[y,x],P1=600,P2=2400),witness=dict(local_BT_costs=c[y,x].tolist(),local_hypothesis_support=m[y,x].tolist(),left_precedent_at_x4_support=m[y,4].tolist(),left_path_costs=p['left'][y,x].tolist(),right_path_costs=p['right'][y,x].tolist(),top_path_costs=p['top'][y,x].tolist(),aggregate_costs=a[y,x].tolist(),decoded_candidate_index=int(w[y,x]),required_UNKNOWN=None),classification='Mathematicallyconsistent support-constrained regularization breaks photometric ties. No unique local color evidence at thispoint. This is failureofdeclared measurementvalidity contract, not proven bug in v118minplus recurrence and not actualrobot false-obstacle performance.',limits='Candidate0 is not0meters and originalpositive depthconsumer rejects disparity0. Actualfour-eye falseoutputs/GTerror/benefit are unmeasured; do not infer them from thisfixture. No confidence/pixelthreshold/P1 variant evaluated.',counts=dict(synthetic_RGB_cost_calls=1,vectorized_aggregate_calls=1,decoder_calls=1,real_RGB_calls=0,GT_reads=0,fit=0,physics=0,navigation=0),array_sha256=sha(D/'failure_analysis_arrays.npz'))
 (D/'failure_analysis_result.json').write_text(json.dumps(v,indent=2,allow_nan=False)+'\n');print('FIXED_CAUSE',json.dumps(v['witness']),flush=True)
if __name__=='__main__':main()
