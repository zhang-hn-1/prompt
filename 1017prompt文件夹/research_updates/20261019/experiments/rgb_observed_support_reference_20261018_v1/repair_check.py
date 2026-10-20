from pathlib import Path
import json,hashlib,datetime
import numpy as np
from evidence_decode import evidence_decode
D=Path(__file__).resolve().parent;sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 cfg=json.loads((D/'repair_protocol.json').read_text())
 for p,h in cfg['sha256'].items():assert sha(p)==h,p
 with np.load(D/'failure_analysis_arrays.npz') as a:cost=a['cost'];support=a['support'];total=a['total'];old=a['decoded']
 before=[x.tobytes() for x in [cost,support,total]];new=evidence_decode(total,cost,support,np.array([0,1]));checks=dict(fixed_prior_boundary_false_confidence_unknown=bool(old[5,6]==0 and np.isnan(new[5,6])),all_flat_fixture_decoded_UNKNOWN=bool(np.isnan(new).all()),saved_arrays_unmodified=before==[x.tobytes() for x in [cost,support,total]])
 # Literal separately chosen interface cases. No RGB/GT/cost-generation/aggregation calls.
 c=np.array([[[100.,0.,1000.],[0.,100.,1000.],[0.,0.,1000.],[0.,100.,1000.],[np.nan,np.nan,np.nan]]])
 t=np.array([[[1000.,0.,9000.],[1000.,0.,9000.],[1000.,0.,9000.],[0.,1000.,9000.],[np.inf,np.inf,np.inf]]])
 m=np.ones(c.shape,bool);m[0,3,1:]=False;m[0,4]=False
 d=evidence_decode(t,c,m,np.array([0,1,2]));checks.update(keeps_agreeing_unique_photometric_choice=bool(d[0,0]==1),rejects_regularizer_unary_disagreement=bool(np.isnan(d[0,1])),rejects_photometric_minimum_tie=bool(np.isnan(d[0,2])),singleton_support_UNKNOWN=bool(np.isnan(d[0,3])),allmissing_UNKNOWN=bool(np.isnan(d[0,4])))
 assert all(checks.values()),checks
 np.savez_compressed(D/'repair_arrays.npz',fixed_old=old,fixed_new=new,literal_cost=c,literal_total=t,literal_support=m,literal_new=d)
 v=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),decision='CONTROLLED_PHOTOMETRIC_EVIDENCE_VALIDITY_REPAIR_SMALL_CHECK_PASSED_NOT_REALRGB',checks=checks,decoder_calls=2,new_RGB_cost_calls=0,new_aggregate_calls=0,real_RGB_calls=0,GT_reads=0,fit=0,physics=0,navigation=0,array_sha256=sha(D/'repair_arrays.npz'),original_failed_gate_unchanged=True,scope='Only newconsumer sanity check. Same originalfailingflatfield savedarrays and five literal interfacecases, no newthreshold. Exactunique localmin and agreement are necessary evidenceconditions, not calibratedconfidence/geometry correctness. No realfourRGB qualification reopened.')
 (D/'repair_result.json').write_text(json.dumps(v,indent=2)+'\n');print('CONTROLLED_REPAIR',checks,flush=True)
if __name__=='__main__':main()
