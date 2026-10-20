"""Author final verification: complete causal reconstruction and immutable identities."""
from pathlib import Path
import datetime,json,hashlib,copy
import numpy as np
from feature_tracks import extract,predict_features,FeatureMemory
from assess_feature_tracks import K
H=Path(__file__).resolve().parent;sha=lambda f:hashlib.sha256(Path(f).read_bytes()).hexdigest()
class FuturePoison(dict):
 def __getitem__(self,key):
  if key!='time_s':raise AssertionError('future payload read')
  return super().__getitem__(key)
def main():
 p=json.loads((H/'feature_tracks_protocol.json').read_text());prior=p.get('previous_artifacts',p.get('prior_artifacts',{}))
 checks={**p['source_sha256'],**prior}
 for f,s in checks.items():assert sha(f)==s,f
 o=H/'feature_tracks_runs';m=json.loads((o/'manifest.json').read_text());a=json.loads((o/'assessment.json').read_text());assert m['labels_not_loaded'] and m['saved_utc']<a['assessed_utc'];assert not a['candidate_accepted']
 masks=np.load(H.parent/'visible_surface_connectivity_20261004_v1/base_link_masks.npz');verified=poison=0;counts={}
 for seq,samples in p['sequences'].items():
  history=[];memory=FeatureMemory(10);counts[seq]={arm:[0,0,0] for arm in ['current','causal']}
  for s in samples:
   i=s['frame'];path=o/seq/f'prediction_{i:03d}.json';r=json.loads(path.read_text());mr=next(v for v in m['rows'] if v['sequence']==seq and v['frame']==i);assert sha(path)==mr['sha256'];assert sha(r['features_file'])==r['features_sha256']
   raw=np.load(s['rgb_file']);f=np.load(r['features_file']);pair=[]
   for eye in ['left','right']:
    xy,desc,size=extract(raw[eye+'_rgb'],masks[eye])
    for k,v in [('xy',xy),('desc',desc),('size',size)]:np.testing.assert_equal(v,f[eye+'_'+k])
    pair.append(dict(frame=i,eye=eye,time_s=r['time_s'],xy=xy,desc=desc,size=size,pose=f[eye+'_pose']))
   for arm,sources in [('current',[pair[1]]),('causal',[pair[1]]+history)]:
    actual=predict_features(pair[0],sources,K);assert actual==r[arm]
    future=FuturePoison(time_s=r['time_s']+1);assert predict_features(pair[0],sources+[future],K)==actual;poison+=1
    labels=[v for v in a['points'] if v['sequence']==seq and v['frame']==i and v['arm']==arm]
    assert len(labels)==len(actual['points']);counts[seq][arm][0]+=actual['query_count'];counts[seq][arm][1]+=len(labels);counts[seq][arm][2]+=sum(v['correct'] for v in labels)
    for pt,lab in zip(actual['points'],labels):
     assert pt['query'][2]==lab['index'];assert pt['query'][3]<=r['time_s'] and pt['source'][3]<=r['time_s']
     if lab['label_world'] is not None:
      err=float(np.linalg.norm(np.array(pt['world'])-lab['label_world']));assert err==lab['error_m'] and (err<=.1)==lab['correct']
   memory.update(r['causal']['points'],r['time_s'],0.);assert memory.snapshot(r['time_s'])==r['memory'];assert not memory.certified_free([0,0],r['time_s'])
   snapshot=memory.snapshot(r['time_s'])
   if snapshot:snapshot[0]['world'][0]+=999;assert memory.snapshot(r['time_s'])==r['memory']
   history=(history+pair)[-6:];verified+=1
  for arm,c in counts[seq].items():
   v=a['sequences'][seq][arm];assert c==[v['queries'],v['retained'],v['correct']];assert not v['gate_pass'];assert abs(v['unconditional_fraction']-(c[2]/c[0]))<1e-15
 report=H.parent.parent/'reports/benchmark_causal_feature_tracks_and_sparse_memory_20261004.md'
 names=['feature_tracks.py','test_feature_tracks.py','run_feature_tracks.py','feature_tracks_protocol.json','feature_tracks_red.log','feature_tracks_green.log','feature_tracks_boundary_red.log','feature_tracks_full_suite.log','feature_tracks_run.log','assess_feature_tracks.py','feature_tracks_assessment.log','audit_feature_tracks.py','feature_tracks_audit.log','show_feature_tracks.py','write_feature_tracks_report.py','verify_feature_tracks.py','feature_tracks_review.json','feature_tracks_verification_prereview.json','feature_tracks_verify.log']
 files=[H/n for n in names if (H/n).exists()]+list(o.rglob('*'))+[report]+list((H/'figures').glob('feature_tracks_*.png'))
 result=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_identity_checks=len(checks),prior_artifact_checks=len(prior),source_checks=len(p['source_sha256']),frames_reconstructed=verified,predictions_reconstructed=2*verified,future_poison_checks=poison,all_feature_arrays_reextracted=True,all_memory_snapshots_reconstructed=True,stage_tests=124,known_pose_only=True,candidate_accepted=False,whole_research_complete=False,actual_new_simulations=0,source_and_artifact_sha256={str(f):sha(f) for f in files if f.is_file()})
 (H/'feature_tracks_verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='source_and_artifact_sha256'},indent=2))
if __name__=='__main__':main()
