"""Author reconstruction; self-output logs excluded from cyclic artifact hashing."""
from pathlib import Path
import json,hashlib,datetime
import numpy as np
from point_support_shadow import PointSupportMap,replay_prefix,swept_indices,support_result
from primitive_planner import assess_support
H=Path(__file__).resolve().parent;O=H/'point_support_runs';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
class FutureTrap(dict):
 def __getitem__(self,key):
  if key!='time_s':raise AssertionError('future payload read')
  return super().__getitem__(key)
def main():
 p=json.loads((H/'point_support_protocol.json').read_text());old=json.loads((H/'active_observation_protocol.json').read_text());i=json.loads((H/'active_feature_protocol.json').read_text());v=json.loads((H/'active_observation_verification.json').read_text());checks={**old['source_sha256'],**old['previous_artifacts'],**i['source_sha256'],**v['source_and_artifact_sha256'],**p['source_sha256']}
 for f,s in checks.items():assert sha(f)==s,f
 m=json.loads((O/'manifest.json').read_text());a=json.loads((O/'assessment.json').read_text());assert p['sealed_utc']<m['saved_utc']<a['assessed_utc'] and m['labels_not_loaded'] and len(m['rows'])==181 and m['maps']==362
 ref=np.load(O/'reference_and_sweep.npz');np.testing.assert_equal(ref['world_xy'],old['preflight']['world_xy']);assert old['sealed_utc']<json.loads((H/'active_observation_status.json').read_text())['start_utc'];origin=np.array(p['grid']['origin']);shape=tuple(p['grid']['shape']);res=p['grid']['resolution'];history=PointSupportMap(origin,shape,res);packets=[];futures=0
 phases=dict(start=ref['world_xy'][:1],turn=ref['world_xy'][:301],stop=ref['world_xy'][300:],whole=ref['world_xy'])
 for phase,path in phases.items():np.testing.assert_equal(swept_indices(path,origin,res,.7),ref[phase+'_cells'])
 for n,row in enumerate(m['rows']):
  maps=np.load(O/f'maps_{n:03d}.npz');saved=json.loads((O/f'frame_{n:03d}.json').read_text());packet=json.loads((O/f'packet_{n:03d}.json').read_text());raw=json.loads((H/'active_feature_runs'/f'prediction_{n:03d}.json').read_text());assert packet['points']==[{k:pt[k] for k in ['world','sigma_m','query','source']} for pt in raw['causal']['points']] and packet['time_s']==raw['time_s'];assert set(packet)=={'frame','time_s','points','pose_sigma_m'}
  for name,key in [(f'maps_{n:03d}.npz','maps_sha256'),(f'frame_{n:03d}.json','record_sha256'),(f'packet_{n:03d}.json','packet_sha256')]:assert sha(O/name)==row[key]
  t=packet['time_s'];current=PointSupportMap(origin,shape,res);current.update(packet['points'],t,0.);history.update(packet['points'],t,0.);packets.append(packet)
  for arm,mapper in [('current',current),('history',history)]:
   s=mapper.snapshot(t);assert s['records']==saved['arms'][arm]['records'] and not s['certified_free'];assert not (maps[arm+'_labels']==0).any()
   for key in ['labels','provenance','floor_time','hazard_time']:np.testing.assert_equal(s[key],maps[arm+'_'+key])
   for phase,path in phases.items():
    r=support_result(s['labels'],ref[phase+'_cells']);assert r==saved['arms'][arm]['support'][phase]
    independent=assess_support(path,s['labels'],origin,res,.7)
    for k in ['checked_cells','unknown_cells','occupied_cells','support_passed']:assert r[k]==independent[k],(n,arm,phase,k)
  truncated=replay_prefix(packets,t,origin,shape,res).snapshot(t);poison=replay_prefix(packets+[FutureTrap(time_s=t+1)],t,origin,shape,res).snapshot(t);futures+=1
  np.testing.assert_equal(truncated['labels'],maps['history_labels']);np.testing.assert_equal(poison['labels'],maps['history_labels']);assert truncated['records']==poison['records']==saved['arms']['history']['records']
  assert s['records']==[{k:pt[k] for k in ['world','sigma_m','query','source']}|{'pose_sigma_m':0.} for pt in raw['memory']]
 assert not a['source_gate_passed'] and not a['consumer_support_gate_passed'];assert a['raw_endpoint_spatial_diagnostic']['all_causal_retained_points']==568 and a['raw_endpoint_spatial_diagnostic']['nominal_xy_inside_whole_sweep']==0
 for arm in ['current','history']:assert a['arms'][arm]['peak_floor_cells']==0 and a['arms'][arm]['whole_min_unknown']==333
 files=list(O.rglob('*'))+list(H.glob('point_support*'))+[H/n for n in ['run_point_support_shadow.py','assess_point_support_shadow.py','show_point_support_shadow.py','write_point_support_report.py','verify_point_support_shadow.py','test_point_support_shadow.py']]+list((H/'figures').glob('point_support*'))+[H.parent.parent/'reports/benchmark_point_support_shadow_20261005.md']
 files=[f for f in files if f.is_file() and f.name!='point_support_verification.json' and not f.name.startswith('point_support_verify')]
 r=dict(verified_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),old_sha_checks=len(checks),cpu_tests=134,maps_reconstructed=362,source_packets_exact=181,old_memory_point_parity=181,future_payload_and_truncate_checks=futures,independent_frozen_support_checks=1448,source_gate_passed=False,consumer_support_gate_passed=False,new_navigation_runs=0,whole_research_complete=False,source_and_artifact_sha256={str(f):sha(f) for f in files})
 (H/'point_support_verification.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='source_and_artifact_sha256'},indent=2))
if __name__=='__main__':main()
