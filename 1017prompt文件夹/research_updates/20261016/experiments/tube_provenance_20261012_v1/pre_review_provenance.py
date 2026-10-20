"""Recorded trace consistency only; not a trusted collector or provenance attestation."""
import hashlib,json,math

def snapshot_digest(sequence_id, observation_times, snapshot_bytes):
 if not isinstance(sequence_id,str) or not sequence_id or type(snapshot_bytes) is not bytes:raise ValueError('sequence and immutable bytes required')
 if not observation_times or not all(math.isfinite(t) for t in observation_times):raise ValueError('finite observation times required')
 header=json.dumps([sequence_id,list(observation_times)],separators=(',',':'),allow_nan=False).encode()
 return hashlib.sha256(len(header).to_bytes(8,'big')+header+snapshot_bytes).hexdigest()

def check_order(*,sequence_id,snapshot_sequence,observation_times,anchor_time,snapshot_bytes,snapshot_sha,commit_event,observation_events,consume_events,future_observation_events):
 def reject(reason):return {'eligible':False,'reason':reason,'trusted_collector_verified':False}
 if sequence_id!=snapshot_sequence:return reject('sequence_mismatch')
 if snapshot_digest(snapshot_sequence,observation_times,snapshot_bytes)!=snapshot_sha:return reject('snapshot_hash_mismatch')
 if not math.isfinite(anchor_time) or not observation_times or observation_times[-1]!=anchor_time or any(t>anchor_time for t in observation_times) or any(a>=b for a,b in zip(observation_times,observation_times[1:])):return reject('future_or_missing_anchor')
 if len(consume_events)!=25:return reject('consumption_count_mismatch')
 if len(observation_events)!=len(observation_times) or not future_observation_events:return reject('event_order_mismatch')
 events=tuple(observation_events)+(commit_event,)+tuple(consume_events)+tuple(future_observation_events)
 if any(type(e) is not int or e<0 for e in events) or any(a>=b for a,b in zip(events,events[1:])):return reject('event_order_mismatch')
 return {'eligible':True,'reason':'recorded_trace_consistent','trusted_collector_verified':False}
