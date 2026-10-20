"""Recompute six baseline metrics/source gaps/latency from sealed outcomes, not handwritten values."""
from pathlib import Path
import json,hashlib
import numpy as np
from export_classical_baseline_registry_v1 import RUNS,summarize
from baseline35_metrics_v1 import assess,observation_gap_diagnostics
H=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if __name__=='__main__':
 old=H/'classical_baseline_registry_v1.json';out=json.loads(old.read_text())
 for layer,names in RUNS.items():
  for record,name in zip(out['layers'][layer],names):
   d=H/name
   manifest=json.loads((d/'artifact_manifest_post_close.json').read_text())['artifact_sha256']
   for file,digest in manifest.items():assert sha(Path(file))==digest,file
   record['original_manifest_items_verified_by_exporter']=len(manifest)
   states=json.loads((d/'states.json').read_text());contacts=json.loads((d/'physics_contacts.json').read_text());episode=json.loads((d/'task_episode.json').read_text())['actual_episode'];rows=json.loads((d/'policy_records.json').read_text());sources=json.loads((d/'source_rows.json').read_text());normalized=[dict(time_s=c['simulation_time_s'],available=c['available'],collision=c['contact']) for c in contacts]
   metrics=assess(states,normalized,episode['start_position_world_m'][:2],episode['goal_position_world_m'][:2]);assert metrics['success']==record['arrived']
   record['recomputed_metrics']=metrics;record['source_gap_diagnostics']=observation_gap_diagnostics(sources,rows)
   times=[float(r['end_to_end_wall_s']) for r in rows if 'end_to_end_wall_s' in r];assert len(times)==len(rows)
   record['end_to_end_wall_latency_s']=dict(samples=len(times),median=float(np.median(times)),p90=float(np.quantile(times,.9)),maximum=max(times),not_real_time_10Hz_claim=True)
   record['input_sha256'].update({str(d/n):sha(d/n) for n in ['policy_records.json','source_rows.json']})
 out.update(schema='classical_stereo_memory_TaskH_baseline_evidence_registry_v3',old_registry_sha256=sha(old),exporter_sha256=sha(Path(__file__)),metric_source_sha256=sha(H/'baseline35_metrics_v1.py'),distinguishes_terminal_capture_from_last_scored_precontact_state=True)
 p=H/'classical_baseline_registry_v3.json';assert not p.exists();p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:[dict(run=r['run'],**r['recomputed_metrics'],latency=r['end_to_end_wall_latency_s']) for r in v] for k,v in out['layers'].items()},indent=2))
