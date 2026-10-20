"""Post-close causal reconstruction of both actually recorded readiness interfaces.

Uses sensor prefix and measured feedback. Does not re-call policy or load GT.
"""
from pathlib import Path
import datetime, hashlib, json, sys
import numpy as np
from observation_handoff import ObservationHandoff
from static_seed_belief import SafeStartBootstrap
from earlier_source_opportunity import EarlierSourceOpportunity
from qualified_hazard_consumer import QualifiedHazardConsumer
from past_point_rgb_memory import PastPointRGBMemory
H=Path(__file__).resolve().parent
sys.path.insert(0,str(H.parent/'oracle_feasible_route_20260929_v9/hybrid'))
from execution_snapshot import ExecutionClone
from roll_pid import bank_from_quaternion
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
status=json.loads((H/'bridge_consumer_stage_status_v1.json').read_text())
assert status['stage']=='ACTUAL_CLOSED_NEEDS_VERIFICATION' and status['returncode']==0
results=[]
class Bomb(dict):
 def __getitem__(self,k):
  if k not in ['time_s','current_appearance_time_s','frame']:raise AssertionError('future payload read')
  return super().__getitem__(k)
def same(a,b):
 assert set(a)==set(b)
 for k in a:np.testing.assert_equal(a[k],b[k])
 return len(a)
for arm in ['C2']:
 entry=status
 D=H/'bridge_consumer_C2_run_v1';seal=D/'artifact_manifest_post_close.json'
 assert sha(seal)==entry['post_close_manifest_sha256']
 hashes=json.loads(seal.read_text())['artifact_sha256']
 assert all(sha(f)==s for f,s in hashes.items())
 states=json.loads((D/'states.json').read_text());rows=json.loads((D/'policy_records.json').read_text())
 qrows=json.loads((D/'hazard_qualification_records.json').read_text());oprows=json.loads((D/'source_opportunity_records.json').read_text())
 def reconstruct_sources(limit=None):
  c=QualifiedHazardConsumer();m=PastPointRGBMemory();o=EarlierSourceOpportunity();ev=None;events={};fields=0;guards=0;n=0
  for qr,orr in zip(qrows,oprows):
   i=qr['frame'];t=qr['time_s']
   if limit is not None and i>limit:break
   src=np.load(D/'sensor_packets'/f'source_{i:03d}.npz');rgb=np.load(D/'online_inputs'/f'frame_{i:03d}.npz');pose=np.load(D/'known_poses'/f'frame_{i:03d}.npz');mask=np.load(D/'masks'/f'frame_{i:03d}.npz')
   low=i>=4 and all(np.linalg.norm(s['velocity'][:2])<.05 for s in states[i-4:i+1])
   w=src['world'];count=int(((w[:,2]>.05)&(w[:,2]<=1.5)).sum())
   pair=o.update(src,np.asarray([s['velocity'][:2] for s in states[max(0,i-4):i+1]]),t,i)
   if pair is not None:ev=dict(time_s=t,frame=i,confirmed=True,pair=pair)
   q=pair is not None
   expected=dict(frame=i,time_s=t,variant=arm,qualified_seed=bool(q),source_event=ev,opportunity_diagnostics=o.diagnostics.copy(),current_diagnostics_valid=bool(o.diagnostics.get('frame')==i))
   assert expected==orr,(arm,i,'opportunity')
   if not m.seeded and q:m.seed(src,rgb,t,qualified=True)
   proof=m.query(rgb,pose,mask,t);saved=np.load(D/'appearance_predictions'/f'frame_{i:03d}.npz')
   fields+=same(proof,{k:saved[k] for k in saved.files if k not in ['frame','time_s']})
   cur,old=c.consume(src,proof,t,q)
   fields+=same(cur,np.load(D/'qualified_sensor_packets'/f'source_{i:03d}.npz'))+same(old,np.load(D/'appearance_old_packets'/f'source_{i:03d}.npz'))
   assert dict(frame=i,**c.diagnostics)==qr
   events[i]=None if ev is None else ev.copy();n+=1
   for s,p in [(Bomb(time_s=t+1),None),(src,Bomb(current_appearance_time_s=t+1))]:
    try:c.consume(s,p,t,False)
    except ValueError:guards+=1
    else:raise AssertionError('future source/appearance accepted')
  return events,fields,guards,n
 events,fields,guards,n=reconstruct_sources()
 def reconstruct_actions(limit=None):
  g=SafeStartBootstrap();outputs=[]
  for i,r in enumerate(rows):
   if limit is not None and i>limit:break
   l=r['local'];t=l['time_s'];sf=l['source_frame'];src=np.load(D/'sensor_packets'/f'source_{sf:03d}.npz')
   assert sf<=i and float(src['time_s'])<=t
   ev=events[sf];nom=np.array(l['nominal_request'],np.float32);v=float(np.linalg.norm(states[i]['velocity'][:2]))
   out=g.choose(t,v,src,nom,ev,i)
   np.testing.assert_array_equal(out,l['request']);np.testing.assert_array_equal(out,np.load(D/'policy_inputs'/f'frame_{i:03d}.npz')['request'])
   assert l['source_gate_variant']==arm and l['observation_handed_off']==g.handed_off and l['source_gate_ready']==g.handed_off and l['observation_low_count']==g.low_count and l['observation_handoff_time_s']==g.handoff_time_s and l['source_readiness_time_s']==g.handoff_time_s
   assert l['source_event_frame']==(ev['frame'] if ev else None) and l['source_event_time_s']==(ev['time_s'] if ev else None)
   assert l['nominal_policy_fields_not_applied']==bool(np.any(out!=nom))
   outputs.append(out)
  return g,outputs
 g,outputs=reconstruct_actions();end=min(len(rows)-1,int(round((g.handoff_time_s or 18)*10))+10)
 pe,pf,pg,pn=reconstruct_sources(end)
 assert pe=={k:v for k,v in events.items() if k<=end}
 _,po=reconstruct_actions(end);np.testing.assert_array_equal(po,outputs[:end+1])
 eventguards=0
 for i in range(10):
  gate=SafeStartBootstrap()
  try:gate.choose(12+i,.0,None,np.zeros(10),Bomb(time_s=13+i,frame=121+10*i),current_frame=120+10*i)
  except ValueError:eventguards+=1
  else:raise AssertionError('future event accepted')
 es=json.loads((D/'execution_states.json').read_text());ph=np.load(D/'physics.npz');N=len(rows)
 assert len(es)==N+1 and es[0] is None
 clone=ExecutionClone.from_snapshot(es[1]);ticks=0
 for tick in range(5,5*N):
  applied,smooth=clone.step(ph['raw_command_target'][tick+1],bank_rad=bank_from_quaternion(ph['root_quaternion_world_wxyz'][tick]),aux_position_rad=ph['joint_position_rad'][tick,1],aux_velocity_rad_s=ph['joint_velocity_rad_s'][tick,1])
  np.testing.assert_equal(applied,ph['applied_action'][tick+1]);np.testing.assert_equal(smooth,ph['smoothed_target'][tick+1])
  if (tick+1)%5==0:assert clone.snapshot()==es[(tick+1)//5]
  ticks+=1
 for i,r in enumerate(rows):np.testing.assert_equal(ph['raw_command_target'][5*i+1:5*i+6],np.asarray(r['local']['request']).reshape(5,2))
 results.append(dict(arm=arm,manifest_checks=len(hashes),source_updates=n,source_fields_exact=fields,future_source_appearance_traps=guards,prefix_source_updates=pn,prefix_fields=pf,actual_requests_exact=N,prefix_requests_exact=len(po),future_event_traps=eventguards,source_readiness_time_s=g.handoff_time_s,actual_handoff_time_s=g.handoff_time_s,confirmed_event=next((e for e in events.values() if e),None),dispatch_ticks=N*5,measured_feedback_clone_ticks=ticks,execution_snapshots=N-1,initial5_not_cloned=True,not_recursive_prediction=True,renderer_labels_read=False,policy_not_recalled=True,archive_wide_future_log_poison_not_claimed=True))
out=H/'bridge_consumer_causal_verification_v1.json';assert not out.exists()
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),arms=results,author_actual_verification=True,code_sha256=sha(__file__),whole_research_complete=False)
out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
