"""Source-only recomputation of sealed decisions, no labels or old assessment."""
from pathlib import Path
import numpy as np,json,hashlib,datetime
from past_geometry_anchor_filter import PastGeometryAnchorFilter
from packet_scalars import normalize_npz_packet
H=Path(__file__).resolve().parent;f=PastGeometryAnchorFilter();run=H/'observation_handoff_observe_run_recovery_v2';out=H/'past_geometry_anchor_predictions_v1';count=0
for p in sorted((run/'sensor_packets').glob('source_*.npz')):
 packet=normalize_npz_packet(dict(np.load(p)));i=int(packet['frame']);d=f.ingest(packet,float(packet['time_s']),i==180);saved=np.load(out/f'frame_{i:03d}.npz')['keep'];np.testing.assert_array_equal(d,saved);count+=1
result=dict(utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),frames_exact=count,reads_only_source_packets_and_saved_outputs=True,prediction_builder_unnecessary_old_assessment_deserialization=True,old_assessment_values_not_used_for_decisions=True,seal_no_offline_truth_access_means_no_frame_label_arrays=True,not_fresh=True)
(H/'past_geometry_source_only_verification_v1.json').write_text(json.dumps(result,indent=2)+'\n');print(result)
