"""Source-only old10s mapper comparison, no navigation or truth decisions."""
from pathlib import Path
import json,numpy as np,datetime,hashlib
from point_support_shadow import PointSupportMap
from dense_source_points import records
from packet_scalars import normalize_npz_packet
H=Path(__file__).resolve().parent;run=H/'observation_handoff_observe_run_recovery_v2';pred=H/'past_geometry_anchor_predictions_v1';states=json.loads((run/'states.json').read_text());origin=np.array(states[0]['position'])[:2]-20;baseline=PointSupportMap(origin,(400,400),.1);candidate=PointSupportMap(origin,(400,400),.1);result=[]
for p in sorted((run/'sensor_packets').glob('source_*.npz')):
 src=normalize_npz_packet(dict(np.load(p)));i=int(src['frame']);t=float(src['time_s']);keep=np.load(pred/f'frame_{i:03d}.npz')['keep'];hz=(src['world'][:,2]>.05)&(src['world'][:,2]<=1.5);retain=~hz|keep;points=records(src);baseline.update(points,t,0.);candidate.update([p for p,k in zip(points,retain) if k],t,0.)
 if i in [180,190,200,210,315]:
  b=baseline.snapshot(t);c=candidate.snapshot(t);assert np.array_equal(b['floor_time'],c['floor_time']);assert not (np.isfinite(c['hazard_time'])&~np.isfinite(b['hazard_time'])).any();result.append(dict(frame=i,baseline_occupied=int((b['labels']==1).sum()),filtered_occupied=int((c['labels']==1).sum()),floor_time_exact=True,original_query_ages_preserved=True,rejected_hazard_can_reveal_existing_floor_support_not_new_free_certificate=True))
(H/'past_geometry_history_map_comparison_v1.json').write_text(json.dumps(dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),rows=result,source_only=True,old_TTL_10s=True,no_map_accuracy_claim=True,no_runtime_installation=True),indent=2)+'\n');print(result)
