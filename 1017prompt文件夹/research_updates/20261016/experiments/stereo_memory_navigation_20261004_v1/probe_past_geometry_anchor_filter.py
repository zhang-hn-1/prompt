"""All causal development predictions sealed before any offline truth access."""
from pathlib import Path
import hashlib,json,datetime,numpy as np
from past_geometry_anchor_filter import PastGeometryAnchorFilter
from packet_scalars import normalize_npz_packet
H=Path(__file__).resolve().parent;run=H/'observation_handoff_observe_run_recovery_v2';out=H/'past_geometry_anchor_predictions_v1';out.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
assessment=json.loads((H/'observation_handoff_observe_assessment_recovery_v2.json').read_text())
# Seed selection frozen before this run; matches archived actual handoff, not a truth lookup.
f=PastGeometryAnchorFilter();inputs={};summaries=[]
for path in sorted((run/'sensor_packets').glob('source_*.npz')):
 packet=normalize_npz_packet(dict(np.load(path)));i=int(packet['frame']);t=float(packet['time_s']);inputs[str(path)]=sha(path)
 keep=f.ingest(packet,t,qualified_seed=i==180)
 np.savez_compressed(out/f'frame_{i:03d}.npz',keep=keep,frame=i,time_s=t,query_pixel_id=packet['query_pixel_id'],partner_pixel_id=packet['partner_pixel_id'],source_model_admitted=False,certified_free=False)
 summaries.append(dict(frame=i,time_s=t,retained=int(keep.sum()),hazards=int(((packet['world'][:,2]>.05)&(packet['world'][:,2]<=1.5)).sum())))
seal=dict(utc=now(),input_SHA256=inputs,prediction_SHA256={str(p):sha(p) for p in sorted(out.glob('*.npz'))},source_SHA256={str(p):sha(p) for p in [Path(__file__),H/'past_geometry_anchor_filter.py',H/'test_past_geometry_anchor_filter.py',H.parent.parent/'docs/superpowers/plans/2026-10-07-past-geometry-anchor-filter.md']},frames=summaries,no_offline_truth_access=True,seed_frame=180,development_not_fresh=True)
(out/'all_predictions_seal.json').write_text(json.dumps(seal,indent=2)+'\n');print(json.dumps(summaries))
