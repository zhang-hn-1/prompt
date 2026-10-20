"""Read-only original-pilot record audit; no fitting or rescoring navigation."""
from pathlib import Path
import json,hashlib,sys
import numpy as np
D=Path(__file__).resolve().parent;R=D/'run_v1'
status=json.loads((D/'run_status_v1.json').read_text());assert status['status']=='finished' and status['child_returncode']==0
manifest=json.loads((D/'supervisor_seal_v1.json').read_text())['artifact_sha256']
assert all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==sha for f,sha in manifest.items())
admission=json.loads((D/'pilot_admission_v1.json').read_text());assert all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==sha for f,sha in admission['source_sha256'].items())
windows=json.loads((R/'windows.json').read_text());terminal=json.loads((R/'terminal.json').read_text());assert len(windows)==70 and all(w['scorable'] for w in windows) and terminal['unattempted_windows']==0
physics=np.load(R/'physics.npz',allow_pickle=False);assert len(physics['simulation_time_s'])==1751
contacts=json.loads((R/'contact_samples.json').read_text());assert all(v['available'] and not v['contact'] for v in contacts)
reads=json.loads((R/'native_reads.json').read_text());assert len(reads)==1751 and all(v['qualified'] for v in reads)
sys.path.insert(0,str(D.parents[0]/'stereo_memory_navigation_20261004_v1'))
from flat_ground_pose_bridge import FlatGroundPoseBridge
gauge=json.loads((R/'initial_gauge.json').read_text());pose=FlatGroundPoseBridge(gauge['position'],gauge['rotation'])
for rec in json.loads((R/'rolling_records.json').read_text()):pose.consume(rec)
count=0;epoch=[]
for i in range(70):
 records=[json.loads(line) for line in (R/f'window_{i:03d}.jsonl').read_text().splitlines()]
 receipts=[r['receipt'] for r in records if r['type']=='anchor_source_receipt'];commits=[r for r in records if r['type']=='commit'];feedback=[r for r in records if r['type']=='block_feedback'];intents=[r for r in records if r['type']=='block_dispatch_intent'];initial=[r for r in records if r['type']=='initial_supervision']
 assert len(receipts)==len(commits)==len(initial)==1 and len(intents)==len(feedback)==5
 receipt=receipts[0];commit=commits[0];envelope=json.loads(bytes.fromhex(commit['snapshot_hex']));state=envelope['online_state'][0];anchor=float(state['time_s']);offset=i*25
 assert abs(anchor-float(physics['simulation_time_s'][offset]))<=1e-7
 assert receipt['clock_metadata'][0]==anchor and receipt['clock_metadata'][2]==receipt['raw_time_s'];epoch.append(receipt['raw_time_s']-anchor)
 assert envelope['source']==dict(source='native_imu_rolling',initial_gauge=True,flat_ground_prior=True)
 for eye in ('left','right'):
  rgb=bytes.fromhex(envelope[eye+'_rgb_hex']);assert len(rgb)==480*640*3 and hashlib.sha256(rgb).hexdigest()==receipt[eye+'_rgb_sha256']
 assert all(pair==[0.,0.] for pair in commit['program'])
 expected=pose.at(anchor);assert np.array_equal(state['estimated_xy'],expected['root_position_world_m'][:2]) and np.array_equal(state['estimated_velocity'],expected['velocity_world_m_s'][:2]) and state['estimated_heading']==expected['yaw_world_rad']
 assert np.array_equal(initial[0]['world_xy'],physics['position_world_xy'][offset])
 assert records.index(commits[0])<records.index(intents[0])
 for block,(intent,actual) in enumerate(zip(intents,feedback)):
  assert records.index(intent)<records.index(actual) and intent['index']==actual['index']==block
  assert len(actual['records'])==5 and not actual['physical_contact'] and not actual['collision']
  for j,rec in enumerate(actual['records']):
   ix=offset+block*5+j+1
   assert rec['actual_raw_pair']==[0.,0.] and np.array_equal(rec['actual_raw_pair'],physics['raw_command_target'][ix])
   assert rec['end_time']==float(physics['simulation_time_s'][ix]) and np.array_equal(rec['end_world_xy'],physics['position_world_xy'][ix]);count+=1
assert count==1750 and max(epoch)-min(epoch)<1e-5
result=dict(classification='interface_record_qualification_pass',windows=70,episode_groups=1,raw_requests=1750,physics_samples=1751,contact_samples=len(contacts),qualified_native_reads=len(reads),sealed_files=len(manifest),source_identities_preserved=True,learning_coverage_qualified=False,trained_model=False,navigation_benefit_assessed=False,pixel_freshness_attested=False,whole_mission_complete=False)
(D/'assessment_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
